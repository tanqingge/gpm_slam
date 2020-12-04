/*
publish laser cloud data
*/

#include "publisher/cloud_publisher.hpp"
#include <pcl/filters/filter.h>
namespace gpm_slam {
CloudPublisher::CloudPublisher(ros::NodeHandle& nh,
                               std::string topic_name,
                               size_t buff_size,
                               std::string frame_id)
    :nh_(nh), frame_id_(frame_id) {
    publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, buff_size);
}

void CloudPublisher::Publish(CloudData::CLOUD_PTR  cloud_ptr_input) {
    sensor_msgs::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::PointCloud2());
    std::vector<int> indicies;
    pcl::removeNaNFromPointCloud(*cloud_ptr_input, *cloud_ptr_input, indicies);
    pcl::toROSMsg(*cloud_ptr_input, *cloud_ptr_output);
    cloud_ptr_output->header.stamp = ros::Time::now();
    cloud_ptr_output->header.frame_id = frame_id_;
    publisher_.publish(*cloud_ptr_output);
}

} // namespace data_output