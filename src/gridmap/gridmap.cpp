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

    void GridMap::setGridBel(int idx,int idy, int val)
    {
        Map_bel_(idx,idy)=val;
    }
    int GridMap::getGridBel(int idx,int idy)
    {
        int Grid_data=Map_bel_(idx,idy);
        return Grid_data;
    };

    //用来记录包含的直线特征穿过了矩阵中的多少个元素，穿过的矩阵元素记为1，返回的int类型为矩阵中不为0的元素数量;   
    int BresenhaminMap(LINE* line_ptr)
    {
        int line_number = line_ptr->size();
        for(int i=0; i<line_number;i++)
        {
            float dy=(*line_ptr[i]).end_point.y-(*line_ptr[i]).start_point.y);
            float dx=(*line_ptr[i]).end_point.x-(*line_ptr[i]).start_point.x);
            if
            if(abs((*line_ptr[i]).k_)<1)
            {
                if(k>0)
            }
            else
            {

            }
        }
    };

    void GridMap::MapInit(LINE* line_ptr)
    {
        cv::Mat Cv_Map_,cv_processedMap;
        BresenhaminMap(line_ptr);
        cv::eigen2cv(Map_bel_,Cv_Map_);
        cv::distanceTransform(Cv_Map_,cv_processedMap,CV_DIST_L2,CV_DIST_MASK_PRECISE);
        cv::cv2eigen(cv_processedMap,Map_bel_);
    }
    
    void GridMap::MapUpdate(LINE* line_ptr)
    {
        Eigen::MatrixXd Old_Map_;
        Old_Map_.resize(size_x,size_y);
        Old_Map_=Map_bel_;
        MapInit(line_ptr);
        Map_bel_=Map_bel_*0.5+Old_Map_*0.5;
    }


        Eigen::Vector2i GetGridId();
        void SetlineinMap(LINE* line_ptr)
        {
            
        };
}