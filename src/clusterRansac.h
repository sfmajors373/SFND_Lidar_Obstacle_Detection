// Using this for the homemade ransac function
#ifndef CLUSTERRANSAC_H_
#define CLUSTERRANSAC_H_

#include <unordered_set>
#include <pcl/common/common.h>
#include "kdtree.h"

std::unordered_set<int> Ransac(typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol);
std::vector<int> proximity(int i, std::vector<std::vector<float>> points, std::vector<int>& cluster,std::vector<bool>& processed_points, KdTree* tree, float distanceTol);
std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol);

#endif