/*
 publish laserscan in ros
 */

#ifndef GPM_SLAM_PUBLISHER_CLOUD_PUBLISHER_HPP_
#define GPM_SLAM_PUBLISHER_CLOUD_PUBLISHER_HPP_

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "gpm_slam/sensor_data/cloud_data.hpp"

namespace gpm_slam {
class CloudPublisher {
  public:
    CloudPublisher(ros::NodeHandle& nh,
                   std::string topic_name,
                   size_t buff_size,
                   std::string frame_id);
    CloudPublisher() = default;
    void Publish(CloudData::CLOUD_PTR cloud_ptr_input);
  
  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_;
};
} 
#endif