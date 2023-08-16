/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "clusterRansac.h"
#include <chrono>
#include <string>
#include <math.h>


std::vector<int> proximity(int i, std::vector<std::vector<float>> points, std::vector<int>& cluster,std::vector<bool>& processed_points, KdTree* tree, float distanceTol)
{
	// mark point as processed
	processed_points.at(i) = true;
	// add point to cluster
	cluster.push_back(i);
	// call kdtree on the point
	std::vector<int> nearby_points = tree->search(points.at(i), distanceTol);
	// iterate through each nearby point
	for (int id : nearby_points)
	{
		// if it has not been processed
		if (!processed_points.at(id))
		{
			// call proximity on it
			proximity(id, points, cluster, processed_points, tree, distanceTol);
		}
	}
	return cluster;
}


std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed_points(points.size(), false);

	// iterate through each point
	// if point not processed
	int i = 0;
	while (i < points.size())
	{
		if (processed_points.at(i))
		{
			i++;
			continue;
		}

		std::vector<int> cluster;
		proximity(i, points, cluster, processed_points, tree, distanceTol);
		clusters.push_back(cluster);
		i++;
	}
 
	return clusters;
}

std::unordered_set<int> Ransac(typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	int cloudSize = cloud->points.size();

	// For max iterations 
	for (int i = 0; i < maxIterations; i++)
	{
		// int currentInliers = 0;
		
		// Randomly sample subset
		std::unordered_set<int> inliers;

		while (inliers.size() < 3)
			inliers.insert(rand() % cloudSize);
		
		float x1, y1, z1, x2, y2, z2, x3, y3, z3;
		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;


		// Fit plane
		// Ax + By + Cz + D = 0
		float line_A = ((y2 - y1)*(z3 - z1)) - ((z2 - z1)*(y3 - y1));
		float line_B = ((z2 - z1)*(x3 - x1)) - ((x2 - x1)*(z3 - z1));
		float line_C = ((x2 - x1)*(y3 - y1)) - ((y2 - y1)*(x3 - x1));
		float line_D = -((line_A * x1) + (line_B * y1) + (line_C * z1));
		

		// Measure distance between every point and fitted line
		for (int j = 0; j < cloudSize; j++)
		{
			// Calculate distance
			float dist = (fabs((line_A * cloud->points[j].x) +
									line_B * cloud->points[j].y +
									line_C * cloud->points[j].z) +
									line_D / 
									(sqrt(line_A * line_A + line_B * line_B + line_C * line_C)));
			// If distance is smaller than threshold count it as inlier
			if (dist <= distanceTol)
			{
				inliers.insert(j);
			}
		}
		// If current inliers more than maxInliers, update inliersResult
		if (inliers.size() > inliersResult.size())
		{
			inliersResult.clear();
			inliersResult = inliers;
		}
	}

	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;
}