#pragma once
#include "object.h"
class ObjPointCloud : public Object {
  public:
    void Init(pcl::PointCloud<PointType>::Ptr cloud);
};