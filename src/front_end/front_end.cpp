#include "front_end/front_end.hpp"

#include <cmath>

namespace gpm_slam
{
    FrontEnd::FrontEnd(const double& resolution,const int& map_width,const int& map_hight)
    :
    current_frame_(resolution, map_width, map_hight)
    {
       
    }

    Eigen::Matrix4f FrontEnd::Update(const LineData line_in_now_,Frame last_key_frame_) 
    {
        if (local_map_frames_.size() == 0) {
            current_frame_.pose = init_pose_;
            current_frame_.LineData=line_in_now_;
            current_frame_.grid_map.MapInit(line_in_now_.line_ptr);
            UpdateNewFrame(current_frame_);
            return current_frame_.pose;
        }

    // 不是第一帧，就正常匹配
    GridMap gridmap_temp=current_frame_.GridMap;
    int r=gridmap_temp.Map_bel.rows();
    int c=gridmap_temp.Map_bel.cols();
    gridmap_temp.Map_bel::Zero();
    float x_min=-0.5+predict_pose_(0,5);
    float x_max=0.5+predict_pose_(0,5);
    float y_min=-0.5+predict_pose_(1,5);
    float y_max=0.5+predict_pose_(1,5);
    float x_last,y_last,theta_last;
    float last_score=0;
    float now_score=0;
    Eigen::Maxrix4f guess_pose=Eigen::Maxrix4f::Identity();
    transform_pose=Eigen::Maxrix4f::Identity();
    float theta_this_frame,x_this_frame,y_this_frame;
    for(float theta_i=-pi/2;theta_i<pi/2;theta_i=theta_i+0.157)
    {
        for(float x=x_min;x<x_max;x=x+0.1)
        {
            for(float y=y_min;y<y_max;y=y+0.1)
            {
                // 更新相邻两帧的相对运动
                guess_pose.block<0,0>(3,3)=Eigen::toRotationMatrix(theta);
                guess_pose.block<0,3>(3,1)<<x,y,0;
                transform_pose=current_frame_.pose.inverse()*guess_pose;
                LINE* temp_line_ptr;
                LineSeg line_tmp;
                for(int i=0;i<line_in_now_.line_ptr->size();i++)
                {
                    Eigen::Vector4f start_p<<(*line_in_now_.line_ptr)[i].start_point.x,(*line_in_now_.line_ptr)[i].start_point.y,(*line_in_now_.line_ptr)[i].start_point.z,1;
                    Eigen::Vector4f end_p<<(*line_in_now_.line_ptr)[i].end_point.x,(*line_in_now_.line_ptr)[i].end_point.y,(*line_in_now_.line_ptr)[i].end_point.z,1;
                    start_p *=guess_pose;
                    end_p *=guess_pose;
                    line_tmp.start_point=start_p;
                    line_tmp.end_point=end_p;
                    temp_line_ptr->push(line_tmp);                   
                }
                gridmap_temp.Bresenham(temp_line_ptr);
                //caculate score
                for(int i =0;i<r;i++)
                {
                    for(int j=0;j<c;j++)
                    {
                        if(gridmap_temp(i,j)!=0)
                        {
                            now_score + = current_frame_.GridMap.Map_bel(i,j);
                        }
                        
                    }
                }
                if (now_score>last_score)
                {
                    theta_this_frame=theta_i;
                    x_this_frame=x;
                    y_this_frame=y;
                }
                last_score=now_score;

            }
        }
    }
    
   

    // 匹配之后根据距离判断是否需要生成新的关键帧，如果需要，则做相应更新
    if (fabs(last_key_frame_pose(0,3) - current_frame_.pose(0,3)) + 
        fabs(last_key_frame_pose(1,3) - current_frame_.pose(1,3)) +
        fabs(last_key_frame_pose(2,3) - current_frame_.pose(2,3)) > 2.0) {
        UpdateNewFrame(current_frame_);
        last_key_frame_pose = current_frame_.pose;
    }

    return current_frame_.pose;
}

    void FrontEnd::SetInitPose(const Eigen::Matrix4f& init_pose)
    {
        init_pose_ = init_pose;
        return true;
    }

    void FrontEnd::SetPredictPose(const Eigen::Matrix4f& last_pose,const Eigen::Matrix4f& now_pose)
    {
        // the last_pose and now_pose are from tf raw data, carculate the trasition and add the change into the predict
        Eigen::Vector3f T_last=last_pose.block<0,3>(3,1);
        Eigen::Vector3f T_now=now_pose.block<0,3>(3,1);
        Eigen::Matrix3f R_last=last_pose.block<0,0>(3,3);
        Eigen::Matrix3f R_now=now_pose.block<0,0>(3,3);
        Eigen::Vector3f delta_t=T_now-T_last;
        Eigen::Matrix3f delta_r=R_last.inverse()*R_now;

        //the predict pose with csm algorithm
        Eigen::Matrix3f predictlast_r=last_pose_.block<0,0>(3,3);
        Eigen::Matrix3f predict_r=delta_r*predictlast_r;
        Eigen::Vector3f predictlast_t=last_pose_.block<0,3>(3,1);
        Eigen::Vector3f predict_t=delta_t+predictlast_t;
        predict_pose_.block<0,0>(3,3)=predict_r;
        predict_pose_.block<0,3>(3,1)=predict_t;
    }

    void FrontEnd::UpdateNewFrame(const Frame& new_key_frame)
    {
        Frame key_frame = new_key_frame;
        local_map_frames_.push_back(key_frame);
        while (local_map_frames_.size() > 20) 
        {
            local_map_frames_.pop_front();
        }

        has_new_local_map_ = true;

    // 更新ndt匹配的目标点云
        if (local_map_frames_.size() < 10) 
        {
            ndt_ptr_->setInputTarget(local_map_ptr_);
        } 
        else 
        {
            CloudData::CLOUD_PTR filtered_local_map_ptr(new CloudData::CLOUD());
            local_map_filter_.setInputCloud(local_map_ptr_);
            local_map_filter_.filter(*filtered_local_map_ptr);
            ndt_ptr_->setInputTarget(filtered_local_map_ptr);
        }

    // 更新全局地图
        global_map_frames_.push_back(key_frame);
        if (global_map_frames_.size() % 100 != 0) 
        {
            return;
        } 
        else 
        {
            global_map_ptr_.reset(new CloudData::CLOUD());
            for (size_t i = 0; i < global_map_frames_.size(); ++i) 
            {
                pcl::transformPointCloud(*global_map_frames_.at(i).cloud_data.cloud_ptr, 
                                    *transformed_cloud_ptr, 
                                    global_map_frames_.at(i).pose);
                *global_map_ptr_ += *transformed_cloud_ptr;
            }
            has_new_global_map_ = true;
        }
    }

    bool FrontEnd::GetCurrentScan(LineData::LineData current_scan)
    {
        line_in_now_.LineData=current_scan;
    }


}


