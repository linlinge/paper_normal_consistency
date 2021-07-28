#pragma once
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "PCLExtend.h"
using namespace std;
using namespace Eigen;
class Object {
  public:
    vector<float> dat_;
    virtual void Init(pcl::PointCloud<PointType>::Ptr cloud);
};