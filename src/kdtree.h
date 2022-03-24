#ifndef KDTREE_H_
#define KDTREE_H_


#include "render/render.h"


// Structure to represent node of kd tree
struct Node {
	std::vector<float> point;
	int id;
	Node* ptr_left;
	Node* ptr_right;

	Node(std::vector<float> arr, int setId)
		: point{ arr }, id{ setId }, ptr_left{ nullptr }, ptr_right{ nullptr } {}

	~Node() {
		delete ptr_left;
		delete ptr_right;
	}
};



struct KdTree {
	Node* ptr_root;

	KdTree() : ptr_root{ nullptr } {}

	~KdTree() { delete ptr_root; }

	void insertHelper(Node** ptrptr_node, unsigned int depth, std::vector<float> point, int id) {

		// Tree in empty
		if (*ptrptr_node == nullptr)
			*ptrptr_node = new Node{ point, id };
		else {
			// Calculate current dim.
			unsigned int cd{ depth % 2 };

			if (point[cd] < ((*ptrptr_node)->point[cd]))
				insertHelper(&((*ptrptr_node)->ptr_left), depth + 1, point, id);
			else
				insertHelper(&((*ptrptr_node)->ptr_right), depth + 1, point, id);
		}
	}

	void insert(std::vector<float> point, int id) {
		// TODO: Fill in this function to insert a new point into the tree.
		// The function should create a new node and place correctly with in the root.

		insertHelper(&ptr_root, 0, point, id);
	}

	void searchHelper(std::vector<float> target, Node* ptr_node, int depth, float distanceTol, std::vector<int>& ids) {
		if (ptr_node != nullptr) {
			if ((ptr_node->point[0] >= (target[0] - distanceTol) && ptr_node->point[0] <= (target[0] + distanceTol))
				&& (ptr_node->point[1] >= (target[1] - distanceTol) && ptr_node->point[1] <= (target[1] + distanceTol))
				&& (ptr_node->point[2] >= (target[2] - distanceTol) && ptr_node->point[2] <= (target[2] + distanceTol))) {

				float distance{
					static_cast<float>(sqrt(
						pow((ptr_node->point[0] - target[0]), 2) + 
						pow((ptr_node->point[1] - target[1]), 2) + 
						pow((ptr_node->point[2] - target[2]), 2)))
				};

				if (distance <= distanceTol)
					ids.push_back(ptr_node->id);
			}

			// Check across boundary.
			if ((target[depth % 3] - distanceTol) < ptr_node->point[depth % 3])
				searchHelper(target, ptr_node->ptr_left, depth + 1, distanceTol, ids);

			if ((target[depth % 3] + distanceTol) > ptr_node->point[depth % 3])
				searchHelper(target, ptr_node->ptr_right, depth + 1, distanceTol, ids);
		}
	}

	// Return a list of point ids in the tree that are within distance of target.
	std::vector<int> search(std::vector<float> target, float distanceTol) {
		
		
		ids;
		searchHelper(target, ptr_root, 0, distanceTol, ids);

		return ids;
	}
};




#endif