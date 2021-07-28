#include "objPointCloud.h"
void ObjPointCloud::Init(pcl::PointCloud<PointType>::Ptr cloud) 
{
    dat_.resize(3 * cloud->points.size());
    for (int i = 0; i < cloud->points.size(); i++) {
        dat_[3 * i + 0] = cloud->points[i].x;
        dat_[3 * i + 1] = cloud->points[i].y;
        dat_[3 * i + 2] = cloud->points[i].z;
    }
}