#include <ros/ros.h>
#include <pcl/common/transforms.h>

#include "subscriber/cloud_subscriber.hpp"
#include "subscriber/line_subscriber.hpp"
#include "tf_listener/tf_listener.hpp"
#include "publisher/cloud_publisher.hpp"
#include "publisher/odometry_publisher.hpp"
#include "publisher/line_publisher.hpp"
#include "publisher/lineonrviz_publisher.hpp"

using namespace gpm_slam;

int int main(int argc, char *argv[])
{
    ros::init(argc, argv, "front_end_node");
    ros::NodeHandle nh_;
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr = std::make_shared<CloudSubscriber>(nh, "scan", 100000);
    std::shared_ptr<LineSubscriber> line_sub_ptr = std::make_shared<LineSubscriber>(nh, "current_scan", 100);
    std::shared_ptr<TFListener> tf_to_odom_ptr = std::make_shared<TFListener>(nh, "odom", "base_link");

    std::shared_ptr<CloudPublisher> cloud_pub_ptr = std::make_shared<CloudPublisher>(nh, "current_scan", 100, "map");
    std::shared_ptr<OdometryPublisher> odom_pub_ptr = std::make_shared<OdometryPublisher>(nh, "lidar_odom", "map", "lidar", 100);
    std::shared_ptr<LinePublisher> line_pub_str=std::make_shared<LinePublisher>(nh,"line_scan",200,"map");
    std::shared_ptr<LineOnRvizPublisher> lineonviz_pub_ptr=std::make_shared<LineOnRvizPublisher>(nh,"line_segments",200,"map");

    std::shared_ptr<FrontEnd> front_end_ptr =std::make_shared<FrontEnd>(0.1,10,10);
    std::deque<CloudData> cloud_data_buff;
    std::deque<LineData> line_data_buff;

    Eigen::Matrix4f tf_to_odom = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f last_pose = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f now_pose = Eigen::Matrix4f::Identity();
    
    bool transform_received = false;
    bool line_subscribe_received=false;
    bool front_end_inited=false;


    static count_tf=0;

    ros::Rate rate(100);
    while (ros::ok())
    {
        /* code for loop body */
        ros::spinOnce();
        //subscribe raw pointcloud data
        cloud_sub_ptr->ParseData(cloud_data_buff);
        //get the initial pose(use tf raw data)
        if(!transform_received)
        {
            if(tf_to_odom_ptr->LookupData(tf_to_odom))
            {
                transform_received=true;
                last_pose=tf_to_odom_ptr->LookupData(tf_to_odom);
                now_pose=tf_to_odom_ptr->LookupData(tf_to_odom);
            }
        }
        else
        {
            while (cloud_data_buff.size() > 0) 
            {
                CloudData cloud_data = cloud_data_buff.front();
                cloud_data_buff.pop_front();
                pcl::transformPointCloud(*cloud_data.cloud_ptr, *cloud_data.cloud_ptr, odometry_matrix);
                cloud_pub_ptr->Publish(cloud_data.cloud_ptr);
                line_sub_ptr->ParseData(line_data_buff);
                while(line_data_buff.size()>0)
                {
                        LineData line_data = line_data_buff.front();
                        line_data_buff.pop_front();
                        
                        now_pose=tf_to_odom_ptr->LookupData(tf_to_odom);
                        if(!front_end_inited)
                        {
                            front_end_inited=true;
                            front_end_ptr->setInitpose(now_pose);
                        }
                        front_end_ptr->setPredictPose(last_pose,now_pose);
                        front_end_ptr->GetCurrentScan(line_data);
                        Eigen::Matrix4f current_pose=front_end_ptr->Update();

                        
                        line_pub_str->Publish(current_scan_ptr);
                        odom_pub_ptr->Publish(current_pose);
                        if (run_time > 460.0 && !has_global_map_published) 
                        {
                            if (front_end_ptr->GetNewGlobalMap(global_map_ptr))
                            {
                                global_map_pub_ptr->Publish(global_map_ptr);
                                has_global_map_published = true;
                            }
                        }
                        last_pose=now_pose;

                }



            }
                
            
        }
    }
    return 0;
}