#ifndef GPM_SLAM_FRONT_END_FRONT_END_HPP_
#define GPM_SLAM_FRONT_END_FRONT_END_HPP_

#include <deque>

#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>

#include "sensor_data/cloud_data.hpp"
#include "sensor_data/line_data.hpp"
#include "gridmap/gridmap.hpp"

namespace lidar_localization {
class FrontEnd {
  public:
    struct Frame { 
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        GridMap grid_map;
        LineData line_in_frame_;
        
    };//key frame

  public:
    FrontEnd();

    Eigen::Matrix4f Update(const CloudData& cloud_data);
    bool SetInitPose(const Eigen::Matrix4f& init_pose);
    bool SetPredictPose(const Eigen::Matrix4f& predict_pose);

    bool GetNewLocalMap(CloudData::CLOUD_PTR& local_map_ptr);
    bool GetNewGlobalMap(CloudData::CLOUD_PTR& global_map_ptr);
    bool GetCurrentScan(CloudData::CLOUD_PTR& current_scan_ptr);

    void SetlineinMap(LINE* line_ptr);
  
  private:
    void UpdateNewFrame(const Frame& new_key_frame);
    void LineMerge();//合并直线，将新的直线加入line_ptr

  private:
    /*pcl::VoxelGrid<CloudData::POINT> cloud_filter_;
    pcl::VoxelGrid<CloudData::POINT> local_map_filter_;
    pcl::VoxelGrid<CloudData::POINT> display_filter_;
    pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>::Ptr ndt_ptr_;//定义NDT匹配方法指针*/

    std::deque<Frame> local_map_frames_;
    std::deque<Frame> global_map_frames_;

    bool has_new_local_map_ = false;
    bool has_new_global_map_ = false;
    CloudData::CLOUD_PTR local_map_ptr_;
    CloudData::CLOUD_PTR global_map_ptr_;
    CloudData::CLOUD_PTR result_cloud_ptr_;
    Frame current_frame_;

    Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f predict_pose_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f last_pose_=Eigen::Matrix4f::Identity();
};
}

#endif