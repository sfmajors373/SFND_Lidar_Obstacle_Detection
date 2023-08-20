/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "clusterRansac.h"
#include <chrono>
#include <string>
#include <math.h>


std::vector<int> proximity(int i, typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::vector<int>& cluster,std::vector<bool>& processed_points, KdTree* tree, float distanceTol)
{
	// mark point as processed
	processed_points.at(i) = true;
	// add point to cluster
	cluster.push_back(i);
	// call kdtree on the point
	std::vector<int> nearby_points = tree->search(cloud->points.at(i), distanceTol);
	// iterate through each nearby point
	for (int id : nearby_points)
	{
		// if it has not been processed
		if (!processed_points.at(id))
		{
			// call proximity on it
			proximity(id, cloud, cluster, processed_points, tree, distanceTol);
		}
	}
	return cluster;
}


std::vector<std::vector<int>> euclideanCluster(typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, KdTree* tree, float distanceTol,
int minSize, int maxSize)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed_points(cloud->points.size(), false);

	// iterate through each point
	// if point not processed
	int i = 0;
	while (i < cloud->points.size())
	{
		if (processed_points.at(i))
		{
			i++;
			continue;
		}

		std::vector<int> cluster;
		proximity(i, cloud, cluster, processed_points, tree, distanceTol);
		if ((cluster.size() >= minSize) && (cluster.size() <= maxSize))
		{
			clusters.push_back(cluster);
		}
		i++;
	}
 
	return clusters;
}

std::unordered_set<int> Ransac(
	typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
	 int maxIterations,
	 float distanceTol)
{
	std::unordered_set<int> inliersResult;
	
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
		int pt1, pt2, pt3;
		auto itr = inliers.begin();
		pt1 = *itr;
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		pt2 = *itr;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;
		pt3 = *itr;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;


		// Fit plane
		// Ax + By + Cz + D = 0
		float line_A = ((y2 - y1)*(z3 - z1)) - ((z2 - z1)*(y3 - y1));
		float line_B = ((z2 - z1)*(x3 - x1)) - ((x2 - x1)*(z3 - z1));
		float line_C = ((x2 - x1)*(y3 - y1)) - ((y2 - y1)*(x3 - x1));
		float line_D = -((line_A * x1) + (line_B * y1) + (line_C * z1));
		float len = (sqrt(line_A * line_A + line_B * line_B + line_C * line_C));
		// std::cout << "a: " << line_A << "B: " << line_B << "C: " << line_C << "D: " << line_D << std::endl;
		

		// Measure distance between every point and fitted line
		for (int j = 0; j < cloudSize; j++)
		{
			if (j != pt1 || j != pt2 || j != pt3)
			{
				// Calculate distance
				float dist = (fabs(((line_A * cloud->points[j].x) +
										line_B * cloud->points[j].y +
										line_C * cloud->points[j].z) +
										line_D) / len);
				// If distance is smaller than threshold count it as inlier
				if (dist <= distanceTol)
				{
					inliers.insert(j);
				}
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

std::unordered_set<int> Ransac2(typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(0);

	pcl::PointXYZI point1;
	pcl::PointXYZI point2;
	pcl::PointXYZI point3;

	int idx1, idx2, idx3;

	float a, b , c, d, dis, len;

	for (int it = 0; it < maxIterations; it++)
	{
		std::unordered_set<int> tempIndices;

		while (tempIndices.size() < 3)
		{
			tempIndices.insert((rand() % cloud->points.size()));
		}

		auto iter = tempIndices.begin();

		idx1 = *iter;
		++iter;
		idx2 = *iter;
		++iter;
		idx3 = *iter;

		point1 = cloud->points[idx1];
		point2 = cloud->points[idx2];
		point3 = cloud->points[idx3];

		a = (((point2.y - point1.y) * (point3.z - point1.z)) - ((point2.z - point1.z) * (point3.y - point1.y)));
        b = (((point2.z - point1.z) * (point3.x - point1.x)) - ((point2.x - point1.x) * (point3.z - point1.z)));
        c = (((point2.x - point1.x) * (point3.y - point1.y)) - ((point2.y - point1.y) * (point3.x - point1.x)));
        d = -(a * point1.x + b * point1.y + c * point1.z);
        len = sqrt(a * a + b * b + c * c);

		std::cout << "A: " << a << "B: " << b << "C: " << c << "D " << d << std::endl;

		for (int pt_cnt = 0; pt_cnt < cloud->points.size(); pt_cnt++)
        {
            if (pt_cnt != idx1 || pt_cnt != idx2 || pt_cnt != idx3)
            {
                dis = (fabs(a * cloud->points[pt_cnt].x + b * cloud->points[pt_cnt].y + c * cloud->points[pt_cnt].z + d) / len);

                // If distance is smaller than threshold count it as inlier
                if (dis <= distanceTol)
                {
                    tempIndices.insert(pt_cnt);
                }
            }
        }

		if (tempIndices.size() > inliersResult.size())
        {
            inliersResult.clear();
            inliersResult = tempIndices;
        }
	}

	return inliersResult;
}