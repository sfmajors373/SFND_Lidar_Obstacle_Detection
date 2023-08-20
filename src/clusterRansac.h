// Using this for the homemade ransac function
#ifndef CLUSTERRANSAC_H_
#define CLUSTERRANSAC_H_

#include <unordered_set>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include "kdtree.h"

std::unordered_set<int> Ransac(typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol);
std::vector<int> proximity(int i, typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::vector<int>& cluster,std::vector<bool>& processed_points, KdTree* tree, float distanceTol);
std::vector<std::vector<int>> euclideanCluster(typename pcl::PointCloud<pcl::PointXYZI>::Ptr points, KdTree* tree, float distanceTol, int minSize, int maxSize);

#endif