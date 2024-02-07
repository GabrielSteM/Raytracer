#include "container.h"
#include <stack>

bool BVH::intersect(Ray ray, double t_min, double t_max, Intersection* hit) {

	std::stack<BVHNode*> lifo;
	lifo.push(root);

	double t = t_max;
	bool no_intersection = true;
	Intersection curr_hit;

	while (!lifo.empty()) {

		BVHNode* curr_node = lifo.top();
		lifo.pop();

		if (curr_node->left || curr_node->right) {

			if (curr_node->aabb.intersect(ray, t_min, t_max)) {

				if (curr_node->right) {
					lifo.push(curr_node->right);
				}
				if (curr_node->left) {
					lifo.push(curr_node->left);
				}
			}			
		} 

		else {

			if (curr_node->aabb.intersect(ray, t_min, t_max)) {
				
				bool intersection = objects[curr_node->idx]->intersect(ray, t_min, t_max, &curr_hit);

				if (intersection && curr_hit.depth <= t) {
					no_intersection = false;
					t = curr_hit.depth;
					*hit = curr_hit;
				}
			}
		}
	}

	if (no_intersection) {
		return false;
	}
	return true;
}

bool Naive::intersect(Ray ray, double t_min, double t_max, Intersection* hit) {
 
	double t = t_max;
	bool no_intersection = true;

	Intersection curr_hit;
	
	for (int i = 0; i < aabbs.size(); i++) {

		if (aabbs[i].intersect(ray, t_min, t_max)) {

			bool intersection = objects[i]->intersect(ray, t_min, t_max, &curr_hit);

			if (intersection && curr_hit.depth <= t) {
				no_intersection = false;
				t = curr_hit.depth;
				*hit = curr_hit;
			}
		}
	}

	if (no_intersection) {
		return false;
	}
	return true;
}
