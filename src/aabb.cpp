#include "aabb.h"


bool AABB::intersect(Ray ray, double t_min, double t_max)  {


	// Référence théorique : https://education.siggraph.org/static/HyperGraph/raytrace/rtinter3.htm
	
	double t_near = -DBL_MAX;
	double t_far = DBL_MAX;

	for (int i = 0; i < 3; i++) {

		if (ray.direction[i] != 0) {

			double direction_frac = 1.0 / ray.direction[i];

			double t_one = (min[i] - ray.origin[i]) * direction_frac;
			double t_two = (max[i] - ray.origin[i]) * direction_frac;

			t_near = std::max(std::min(t_one, t_two), t_near);
			t_far = std::min(std::max(t_one, t_two), t_far);

			if (t_near > t_far || t_far < t_min || t_near > t_max) {
				return false;
			}
		}

		else if (ray.origin[i] < min[i] || ray.origin[i] > max[i]) {
			return false;
		}
	}

	return true;

};

std::vector<double3> retrieve_corners(AABB aabb) {

	double x_max = aabb.max[0];
	double y_max = aabb.max[1];
	double z_max = aabb.max[2];
	double x_min = aabb.min[0];
	double y_min = aabb.min[1];
	double z_min = aabb.min[2];
	
	double3 p1 = {x_max, y_max, z_max};
	double3 p2 = {x_max, y_max, z_min};
	double3 p3 = {x_max, y_min, z_max};
	double3 p4 = {x_max, y_min, z_min};
	double3 p5 = {x_min, y_max, z_max};
	double3 p6 = {x_min, y_max, z_min};
	double3 p7 = {x_min, y_min, z_max};
	double3 p8 = {x_min, y_min, z_min};

	return std::vector<double3>{p1, p2, p3, p4, p5, p6, p7, p8};
};

AABB construct_aabb(std::vector<double3> points) {

	double x_max = -DBL_MAX;
	double y_max = -DBL_MAX;
	double z_max = -DBL_MAX;
	double x_min = DBL_MAX;
	double y_min = DBL_MAX;
	double z_min = DBL_MAX;

	for (int i = 0; i < points.size(); i++) {
		x_max = points[i][0] > x_max ? points[i][0] : x_max;
		y_max = points[i][1] > y_max ? points[i][1] : y_max;
		z_max = points[i][2] > z_max ? points[i][2] : z_max;
		x_min = points[i][0] < x_min ? points[i][0] : x_min;
		y_min = points[i][1] < y_min ? points[i][1] : y_min;
		z_min = points[i][2] < z_min ? points[i][2] : z_min;
	}

	return AABB{double3{x_min, y_min, z_min}, double3{x_max, y_max, z_max}};
};

AABB combine(AABB a, AABB b) {
	return AABB{
				{
					min(a.min,b.min)
				},
				{
					max(a.max,b.max)
				}};
};

bool compare(AABB a, AABB b, int axis){
	return a.min[axis] < b.min[axis];
};