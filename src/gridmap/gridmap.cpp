#include "gridmap/gridmap.hpp"
#include <opencv2/core/eigen.hpp>

namespace gpm_slam
{
    GridMap::GridMap(const double& resolution,const int& map_width,const int& map_hight):
    resolution_(resolution),map_width_(map_width),map_height_(map_hight)
    {
        int size_x=map_width_/resolution_+1;
        int size_y=map_height_/resolution_+1;
        Map_bel_.resize(size_x,size_y);
        Map_bel_::zero();
        init_x_=size_x/2;
        init_y_=size_y/2;
    }

    void GridMap::setGridBel();
    int GridMap::getGridBel(int idx,int idy)
    {
        int Grid_data=Map_bel_(idx,idy);
        return Grid_data;
    };

    //用来记录包含的直线特征穿过了矩阵中的多少个元素，穿过的矩阵元素记为1，返回的int类型为矩阵中不为0的元素数量;   
    int BresenhaminMap();
    void MapInit()
    {
        cv::Mat Cv_Map_;

    }
        void MapUpdate();
        Eigen::Vector2i GetGridId();
        void SetlineinMap();
}