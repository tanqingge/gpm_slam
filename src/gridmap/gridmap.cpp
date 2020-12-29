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
        Map_bel_::ones();
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
            dy=(int)std::abs(dy);
            dx=(int)std::abs(dx);
            int x1=std::round((*line_ptr[i]).start_point.x);
            int x2=std::round((*line_ptr[i]).end_point.x);
            int y1=std::round((*line_ptr[i]).start_point.y);
            int y2=std::round((*line_ptr[i]).end_point.y);
            int sign1 = x2-x1?1:-1;
            int sign2 = y2-y1?1:-1;
            //直线近似于垂直情况
            if(x1==x2)
            {
                int y_min=y1>y2?y1:y2;
                int y_max=y1>y2?y2:y1;
                for(int i=y_min;i<y_max;i++)
                setGridBel(x1,i,0);
            }
            //剩下的情况
            bool interchange=false;//用来统计xy是否需要交换
            if(dx<dy)
            {
                interchange=true;
                std::swap(x1,y1);
                std::swap(x2,y2);
            }
            //保证x的起始点永远小于终止点，是++
            if(x1>x2){ 
                std::swap(x1,x2);
                std::swap(y1,y2);
            }
            int increase=y1<y2?1:-1;
            int r1=2*dy;
            int r2=r1-2*dx;
            int p=r1 -dx;
            interchange?setGridBel(y1+init_x_,x1+init_y_,0):setGridBel(x1+init_x_,y1+init_y_,0);
            int x=x1,y=y1;
            while(x<x2)
            {
                x++;
                if(p>0)
                {
                    p+ =r2;
                    y + =increase;
                    interchange?setGridBel(y1+init_x_,x1+init_y_,0):setGridBel(x1+init_x_,y1+init_y_,0);
                }
                else
                {
                    p += r1;
                    interchange?setGridBel(y1+init_x_,x1+init_y_,0):setGridBel(x1+init_x_,y1+init_y_,0);
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


        Eigen::Vector2i GetGridId()
        {
        };

}