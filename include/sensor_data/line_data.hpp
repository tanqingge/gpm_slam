#ifndef GPM_SLAM_SENSOR_DATA_LINE_DATA_HPP_
#define GPM_SLAM_SENSOR_DATA_LINE_DATA_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace gpm_slam {
class CloudData {
  public:
    using POINT = pcl::PointXY;
    using CLOUD = pcl::PointCloud<POINT>;
    using CLOUD_PTR = CLOUD::Ptr;

  public:
    CloudData()
      :cloud_ptr(new CLOUD()) {
    }

  public:
    double time = 0.0;
    CLOUD_PTR cloud_ptr;
};
}

#endif