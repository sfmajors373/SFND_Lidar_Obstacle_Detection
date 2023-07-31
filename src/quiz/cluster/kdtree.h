/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <math.h>

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper(Node *&node, uint depth, std::vector<float> point, int id)
	{
		if (node == NULL)
		{
			node = new Node(point, id);
		}
		else
		{
			uint comparator = depth % point.size();
			if (point[comparator] < node->point[comparator])
			{
				insertHelper((node->left), depth+1, point, id);
			}
			else
			{
				insertHelper((node->right), depth+1, point, id);
			}
			
		}
	}


	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(root, 0, point, id);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		// std::cout << "\n Target: " << target.at(0) << ", " << target.at(1) << std::endl;
		searchHelper(target, distanceTol, root, 0, ids);
		return ids;
	}
	
	void searchHelper(std::vector<float> target, float distanceTol, Node *currentNode, int depth, std::vector<int>& ids)
	{
		// std::cout << "\n Target: " << target.at(0) << ", " << target.at(1) << std::endl;
		// std::cout << "Point to check: " << currentNode->point.at(0) << ", " << currentNode->point.at(1) << std::endl;

 		if (currentNode == nullptr) return;
		// std::cout << "\n Target: " << target.at(0) << ", " << target.at(1) << std::endl;
		// std::cout << "Point to check: " << currentNode->point.at(0) << ", " << currentNode->point.at(1) << std::endl;

		float x1 = target.at(0);
		float y1 = target.at(1);

		float x2 = currentNode->point.at(0);
		float y2 = currentNode->point.at(1);

		int compNum = depth % 2;

		if ((x2 >= (x1 - distanceTol) && x2 <= (x1 + distanceTol)) &&
		(y2 >= (y1 - distanceTol) && y2 <= (y1 + distanceTol)))
		{
			// std::cout << "In box" << std::endl;
			float dist = sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
			if (dist <= distanceTol)
			{
				// std::cout << "In tolerance" << std::endl;
				ids.push_back(currentNode->id);
			}
		}
		if ((target[compNum] - distanceTol) < currentNode->point[compNum])
		{
			// std::cout << "To the left" << std::endl;
			searchHelper(target, distanceTol, currentNode->left, (compNum+1), ids);
		}
		if ((target[compNum] + distanceTol) > currentNode->point[compNum])
		{
			// std::cout << "To the right" << std::endl;
			searchHelper(target, distanceTol, currentNode->right, (compNum+1), ids);
		}
		// TODO: figure out what to do if it equals
	}
};




