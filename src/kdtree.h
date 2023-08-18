/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "render/render.h"
#include <math.h>

// Structure to represent node of kd tree
struct Node
{
	pcl::PointXYZI point;
	int id;
	Node* left;
	Node* right;

	Node(pcl::PointXYZI arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper(Node *&node, uint depth, pcl::PointXYZI point, int id)
	{
		if (node == NULL)
		{
			node = new Node(point, id);
		}
		else
		{
			uint compNum = depth % 3;
			float compVal;
			float compPoint;

			if (compNum == 0)
			{
				compVal = point.x;
				compPoint = node->point.x;
			}
			else if (compNum == 1)
			{
				compVal = point.y;
				compPoint = node->point.y;
			}
			else if (compNum == 2)
			{
				compVal = point.z;
				compPoint = node->point.z;
			}

			if (compVal < compPoint)
			{
				insertHelper((node->left), depth+1, point, id);
			}
			else
			{
				insertHelper((node->right), depth+1, point, id);
			}
			
		}
	}


	void insert(pcl::PointXYZI point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(root, 0, point, id);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(pcl::PointXYZI target, float distanceTol)
	{
		std::vector<int> ids;
		// std::cout << "\n Target: " << target.at(0) << ", " << target.at(1) << std::endl;
		searchHelper(target, distanceTol, root, 0, ids);
		return ids;
	}
	
	void searchHelper(pcl::PointXYZI target, float distanceTol, Node *currentNode, int depth, std::vector<int>& ids)
	{
		// std::cout << "\n Target: " << target.at(0) << ", " << target.at(1) << std::endl;
		// std::cout << "Point to check: " << currentNode->point.at(0) << ", " << currentNode->point.at(1) << std::endl;

 		if (currentNode == nullptr) return;
		// std::cout << "\n Target: " << target.at(0) << ", " << target.at(1) << std::endl;
		// std::cout << "Point to check: " << currentNode->point.at(0) << ", " << currentNode->point.at(1) << std::endl;

		float x1 = target.x;
		float y1 = target.y;
		float z1 = target.z;

		float x2 = currentNode->point.x;
		float y2 = currentNode->point.y;
		float z2 = currentNode->point.z;

		int compNum = depth % 3;

		float compVal;
		float compPoint;

		if (compNum == 0)
		{
			compVal = target.x;
			compPoint = currentNode->point.x;
		}
		else if (compNum == 1)
		{
			compVal = target.y;
			compPoint = currentNode->point.y;
		}
		else if (compNum == 2)
		{
			compVal = target.z;
			compPoint = currentNode->point.z;
		}

		if ((x2 >= (x1 - distanceTol) && x2 <= (x1 + distanceTol)) &&
		(y2 >= (y1 - distanceTol) && y2 <= (y1 + distanceTol)) &&
		(z2 >= (z1 - distanceTol) && z2 <= (z1 + distanceTol)))
		{
			// std::cout << "In box" << std::endl;
			float dist = sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) + (z2-z1)*(z2-z1));
			if (dist <= distanceTol)
			{
				// std::cout << "In tolerance" << std::endl;
				ids.push_back(currentNode->id);
			}
		}
		if ((compVal - distanceTol) < compPoint)
		{
			// std::cout << "To the left" << std::endl;
			searchHelper(target, distanceTol, currentNode->left, (depth+1), ids);
		}
		if ((compVal + distanceTol) > compPoint)
		{
			// std::cout << "To the right" << std::endl;
			searchHelper(target, distanceTol, currentNode->right, (depth+1), ids);
		}
		// TODO: figure out what to do if it equals
	}
};




