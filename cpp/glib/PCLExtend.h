#pragma once
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/octree/octree.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <omp.h>
#include <vector>
#include <Eigen/Dense>
#ifndef PointType
#define PointType pcl::PointXYZRGBNormal
#endif
using namespace std;
using namespace Eigen;
double ComputeMeanDistance(const pcl::PointCloud<PointType>::ConstPtr cloud);
double ComputeMaxDistance(const pcl::PointCloud<PointType>::ConstPtr cloud);
void TransformPointCloud(pcl::PointCloud<PointType>::Ptr cloud, pcl::PointCloud<PointType>::Ptr cloud_tf,Eigen::Affine3f tf);
vector<double> StatisticNearestDistance(const pcl::PointCloud<PointType>::ConstPtr cloud);
double GetCellSize(pcl::PointCloud<PointType>::Ptr cloud, int level);
double GetBoxMin(pcl::PointCloud<PointType>::Ptr cloud);

/* Algorithm Section */
vector<int> LoOP(pcl::PointCloud<PointType>::Ptr cloud_in, int K=45, double thresh=0.8);
void RecoverColor(pcl::PointCloud<PointType>::Ptr cloud_without_color,pcl::PointCloud<PointType>::Ptr cloud_with_color);
void subtract_points(pcl::PointCloud<PointType>::Ptr cloud, const vector<int>& indices, bool flag=false);
void FindCorrespondingIndices(pcl::search::KdTree<PointType>::Ptr kdtree, pcl::PointCloud<PointType>::Ptr acloud, vector<int>& indices);
void ComputeCentre(pcl::PointCloud<PointType>::Ptr cloud, float& cx, float& cy, float& cz);
vector<Vector3f> get_corners(Vector3f& vmin, Vector3f& vmax);