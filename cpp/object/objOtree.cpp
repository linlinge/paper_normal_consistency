#include "objOtree.h"

void ObjOtree::Init(pcl::PointCloud<PointType>::Ptr cloud)
{
    //最小体素的边长
    float resolution = 10;
    pcl::octree::OctreePointCloudSearch<PointType> octree(resolution);
    octree.setInputCloud(cloud);
    // 从输入点云构建八叉树
    octree.addPointsFromInputCloud();
    //求出体素边界
    int depth = octree.getTreeDepth();
    int leaf_size=0;
    for (auto it = octree.begin(depth); it != octree.end(); ++it) {
        if (it.isLeafNode()) {
            leaf_size++;
        }
    }
    dat_.resize(leaf_size * 72);
    int count=0;
    for (auto it = octree.begin(depth); it != octree.end(); ++it) {
        if (it.isLeafNode()) {
            // Get minimum and maximum boundary of each voxel
            Eigen::Vector3f voxel_min, voxel_max;
            octree.getVoxelBounds(it, voxel_min, voxel_max);
            vector<Vector3f> c = get_corners(voxel_min, voxel_max);
            int i = count++;
            /* lines 1*/
            dat_[72 * i + 0] = c[0][0];
            dat_[72 * i + 1] = c[0][1];
            dat_[72 * i + 2] = c[0][2];
            dat_[72 * i + 3] = c[1][0];
            dat_[72 * i + 4] = c[1][1];
            dat_[72 * i + 5] = c[1][2];
            /* lines 2*/
            dat_[72 * i + 6] = c[1][0];
            dat_[72 * i + 7] = c[1][1];
            dat_[72 * i + 8] = c[1][2];
            dat_[72 * i + 9] = c[2][0];
            dat_[72 * i + 10] = c[2][1];
            dat_[72 * i + 11] = c[2][2];
            /* lines 3*/
            dat_[72 * i + 12] = c[2][0];
            dat_[72 * i + 13] = c[2][1];
            dat_[72 * i + 14] = c[2][2];
            dat_[72 * i + 15] = c[3][0];
            dat_[72 * i + 16] = c[3][1];
            dat_[72 * i + 17] = c[3][2];
            /* lines 4*/
            dat_[72 * i + 18] = c[3][0];
            dat_[72 * i + 19] = c[3][1];
            dat_[72 * i + 20] = c[3][2];
            dat_[72 * i + 21] = c[0][0];
            dat_[72 * i + 22] = c[0][1];
            dat_[72 * i + 23] = c[0][2];
            /* lines 5*/
            dat_[72 * i + 12] = c[4][0];
            dat_[72 * i + 13] = c[4][1];
            dat_[72 * i + 14] = c[4][2];
            dat_[72 * i + 15] = c[5][0];
            dat_[72 * i + 16] = c[5][1];
            dat_[72 * i + 17] = c[5][2];
            /* lines 6*/
            dat_[72 * i + 12] = c[5][0];
            dat_[72 * i + 13] = c[5][1];
            dat_[72 * i + 14] = c[5][2];
            dat_[72 * i + 15] = c[6][0];
            dat_[72 * i + 16] = c[6][1];
            dat_[72 * i + 17] = c[6][2];
            /* lines 7*/
            dat_[72 * i + 12] = c[6][0];
            dat_[72 * i + 13] = c[6][1];
            dat_[72 * i + 14] = c[6][2];
            dat_[72 * i + 15] = c[7][0];
            dat_[72 * i + 16] = c[7][1];
            dat_[72 * i + 17] = c[7][2];
            /* lines 8*/
            dat_[72 * i + 12] = c[7][0];
            dat_[72 * i + 13] = c[7][1];
            dat_[72 * i + 14] = c[7][2];
            dat_[72 * i + 15] = c[4][0];
            dat_[72 * i + 16] = c[4][1];
            dat_[72 * i + 17] = c[4][2];
            /* lines 9*/
            dat_[72 * i + 12] = c[0][0];
            dat_[72 * i + 13] = c[0][1];
            dat_[72 * i + 14] = c[0][2];
            dat_[72 * i + 15] = c[4][0];
            dat_[72 * i + 16] = c[4][1];
            dat_[72 * i + 17] = c[4][2];
            /* lines 10*/
            dat_[72 * i + 12] = c[1][0];
            dat_[72 * i + 13] = c[1][1];
            dat_[72 * i + 14] = c[1][2];
            dat_[72 * i + 15] = c[5][0];
            dat_[72 * i + 16] = c[5][1];
            dat_[72 * i + 17] = c[5][2];
            /* lines 11*/
            dat_[72 * i + 12] = c[2][0];
            dat_[72 * i + 13] = c[2][1];
            dat_[72 * i + 14] = c[2][2];
            dat_[72 * i + 15] = c[6][0];
            dat_[72 * i + 16] = c[6][1];
            dat_[72 * i + 17] = c[6][2];
            /* lines 12*/
            dat_[72 * i + 12] = c[3][0];
            dat_[72 * i + 13] = c[3][1];
            dat_[72 * i + 14] = c[3][2];
            dat_[72 * i + 15] = c[7][0];
            dat_[72 * i + 16] = c[7][1];
            dat_[72 * i + 17] = c[7][2];
        }
    }
}