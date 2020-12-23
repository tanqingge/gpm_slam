#ifndef GPM_SLAM_GRIDMAP_GRIDMAP_HPP_
#define GPM_SLAM_PUBLISHER_CLOUD_PUBLISHER_HPP_

#include <eigen3/Eigen/Core>
#include <nav_msgs/OccupancyGrid.h>
#include "sensor_data/line_data.hpp"

namespace gpm_slam
{
    /* code */
    class GridMap
    {
    public:
        GridMap(arguments);
        virtual ~GridMap();

    public:
        void setGridBel();
        void getGridBel(int idx,int idy);
        void MapInit();
        void MapUpdate();
        Eigen::Vector2i GetGridId();
        void SetlineinMap();
        int BresenhaminMap();
    private:
        /* data */
        double resolution_;
        int map_width_,map_height_;//the size of map
        int init_x_,init_y_;//the left border of map
        Eigen::MatrixXd Map_bel_;

    };
    
}


#endif