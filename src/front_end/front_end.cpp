#include "front_end/front_end.hpp"

#include <cmath>

namespace gpm_slam
{
    FrontEnd::FrontEnd(const double& resolution,const int& map_width,const int& map_hight)
    {
        local_map_ptr_(new CloudData::CLOUD());
        global_map_ptr_(new CloudData::CLOUD());
        result_cloud_ptr_(new CloudData::CLOUD());
        current_frame_.GridMap=GridMap(resolution, map_width, map_hight);
    }

    Eigen::Matrix4f FrontEnd::Update(const CloudData& cloud_data) 
    {
        /*current_frame_.cloud_data.time = cloud_data.time;
    
        static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
        static Eigen::Matrix4f last_pose = init_pose_;
        static Eigen::Matrix4f predict_pose = init_pose_;
        static Eigen::Matrix4f last_key_frame_pose = init_pose_;*/
        

    // 局部地图容器中没有关键帧，代表是第一帧数据
    // 此时把当前帧数据作为第一个关键帧，并更新局部地图容器和全局地图容器
        if (local_map_frames_.size() == 0) {
            current_frame_.pose = init_pose_;
            UpdateNewFrame(current_frame_);
            return current_frame_.pose;
        }

    // 不是第一帧，就正常匹配
    ndt_ptr_->setInputSource(filtered_cloud_ptr);
    ndt_ptr_->align(*result_cloud_ptr_, predict_pose);
    current_frame_.pose = ndt_ptr_->getFinalTransformation();

    // 更新相邻两帧的相对运动
    step_pose = last_pose.inverse() * current_frame_.pose;
    predict_pose = current_frame_.pose * step_pose;
    last_pose = current_frame_.pose;

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
    // 这一步的目的是为了把关键帧的点云保存下来
    // 由于用的是共享指针，所以直接复制只是复制了一个指针而已
    // 此时无论你放多少个关键帧在容器里，这些关键帧点云指针都是指向的同一个点云
    key_frame.cloud_data.cloud_ptr.reset(new CloudData::CLOUD(*new_key_frame.cloud_data.cloud_ptr));
    CloudData::CLOUD_PTR transformed_cloud_ptr(new CloudData::CLOUD());
    
    // 更新局部地图
    local_map_frames_.push_back(key_frame);
    while (local_map_frames_.size() > 20) {
        local_map_frames_.pop_front();
    }
    local_map_ptr_.reset(new CloudData::CLOUD());
    for (size_t i = 0; i < local_map_frames_.size(); ++i) {
        pcl::transformPointCloud(*local_map_frames_.at(i).cloud_data.cloud_ptr, 
                                 *transformed_cloud_ptr, 
                                 local_map_frames_.at(i).pose);
        *local_map_ptr_ += *transformed_cloud_ptr;
    }
    has_new_local_map_ = true;

    // 更新ndt匹配的目标点云
    if (local_map_frames_.size() < 10) {
        ndt_ptr_->setInputTarget(local_map_ptr_);
    } else {
        CloudData::CLOUD_PTR filtered_local_map_ptr(new CloudData::CLOUD());
        local_map_filter_.setInputCloud(local_map_ptr_);
        local_map_filter_.filter(*filtered_local_map_ptr);
        ndt_ptr_->setInputTarget(filtered_local_map_ptr);
    }

    // 更新全局地图
    global_map_frames_.push_back(key_frame);
    if (global_map_frames_.size() % 100 != 0) {
        return;
    } else {
        global_map_ptr_.reset(new CloudData::CLOUD());
        for (size_t i = 0; i < global_map_frames_.size(); ++i) {
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
        current_frame_.line_in_frame_=current_scan;       
    }


}


