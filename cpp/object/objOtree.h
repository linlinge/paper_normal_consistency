#pragma once
#include "object.h"
class ObjOtree : public Object {
  public:
    void Init(pcl::PointCloud<PointType>::Ptr cloud);
};