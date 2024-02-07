#include "object.h"

int rsign(double value, double v0, double v1) {
	return (int(std::signbit(value)) * (v1-v0)) + v0;
}

bool Sphere::local_intersect(Ray ray, double t_min, double t_max, Intersection *hit) {

	double a = dot(ray.direction, ray.direction);
	double b = 2.0 * dot(ray.origin, ray.direction);
	double c = dot(ray.origin, ray.origin) - radius * radius;

	double discriminant = b * b - 4.0 * a * c;

	if (discriminant >= 0 && a > 0) {

		double t_one = (-b - sqrt(discriminant))/(2.0 * a);
		double t_two = (-b + sqrt(discriminant))/(2.0 * a);

		bool t_one_in_bounds = t_one >= t_min && t_one <= t_max;
		bool t_two_in_bounds = t_two >= t_min && t_two <= t_max;

		if (!t_one_in_bounds && !t_two_in_bounds) {
			return false;
		}

		else if (!t_one_in_bounds && t_two_in_bounds) {
			hit->depth = t_two;
			hit->key_material = key_material;
			hit->position = ray.origin + hit->depth * ray.direction;
			hit->normal = -hit->position;
			hit->uv = double2{0.5 - atan2(hit->position[0], -hit->position[2])/(2.0 * PI), acos(hit->position[1]/radius)/(PI)};
			return true;
		}

		else if (t_one_in_bounds && !t_two_in_bounds) {
			hit->depth = t_one;
			hit->key_material = key_material;
			hit->position = ray.origin + hit->depth * ray.direction;
			hit->normal = hit->position;
			hit->uv = double2{0.5 - atan2(hit->position[0], -hit->position[2])/(2.0 * PI), acos(hit->position[1]/radius)/(PI)};
			return true;
		}

		else {
			hit->depth = t_one;
			hit->key_material = key_material;
			hit->position = ray.origin + hit->depth * ray.direction;
			hit->normal = hit->position;
			hit->uv = double2{0.5 - atan2(hit->position[0], -hit->position[2])/(2.0 * PI), acos(hit->position[1]/radius)/(PI)};
			return true;
		}
	}
	return false;
}

AABB Sphere::compute_aabb() {

	double3 p1 = mul(transform,{ radius,  radius,  radius, 1}).xyz();
	double3 p2 = mul(transform,{ radius,  radius, -radius, 1}).xyz();
	double3 p3 = mul(transform,{ radius, -radius,  radius, 1}).xyz();
	double3 p4 = mul(transform,{ radius, -radius, -radius, 1}).xyz();
	double3 p5 = mul(transform,{-radius,  radius,  radius, 1}).xyz();
	double3 p6 = mul(transform,{-radius,  radius, -radius, 1}).xyz();
	double3 p7 = mul(transform,{-radius, -radius,  radius, 1}).xyz();
	double3 p8 = mul(transform,{-radius, -radius, -radius, 1}).xyz();

	std::vector<double3> points = std::vector<double3>{p1, p2, p3, p4, p5, p6, p7, p8};

	return construct_aabb(points);
}

bool Quad::local_intersect(Ray ray, double t_min, double t_max, Intersection *hit) {

	double ray_origin_z = ray.origin[2];
	double ray_dir_z = ray.direction[2];

	if (ray_dir_z != 0) {
		double t = -ray_origin_z/ray_dir_z;

		if (t < t_min || t > t_max) {
			return false;
		}

		double3 position = ray.origin + t * ray.direction;

		if (fabs(position[0]) < half_size && fabs(position[1]) < half_size) {
			hit->depth = t;
			hit->key_material = key_material;
			hit->position = position;

			if (ray.origin[2] > 0) {
				hit->normal = double3{0,0,1};
			}
			else if (ray.origin[2] < 0) {
				hit->normal = double3{0,0,-1};
			}

			double u = 1/(2.0 * half_size);
			double v = 1/(2.0 * half_size);
			hit->uv = double2{(hit->position[0] + half_size) * u, -(hit->position[1] - half_size) * v};
			return true;
		}
	}

	return false;
}

AABB Quad::compute_aabb() {
	

	double3 p1 = mul(transform,{ half_size,  half_size, 0, 1}).xyz();
	double3 p2 = mul(transform,{ half_size, -half_size, 0, 1}).xyz();
	double3 p3 = mul(transform,{-half_size,  half_size, 0, 1}).xyz();
	double3 p4 = mul(transform,{-half_size, -half_size, 0, 1}).xyz();
	double3 p5 = mul(transform,{ half_size,  half_size, EPSILON, 1}).xyz();
	double3 p6 = mul(transform,{ half_size, -half_size, EPSILON, 1}).xyz();
	double3 p7 = mul(transform,{-half_size,  half_size, EPSILON, 1}).xyz();
	double3 p8 = mul(transform,{-half_size, -half_size, EPSILON, 1}).xyz();

	std::vector<double3> points = std::vector<double3>{p1, p2, p3, p4, p5, p6, p7, p8};

	return construct_aabb(points);
}

bool Cylinder::local_intersect(Ray ray, double t_min, double t_max, Intersection *hit) {

	double2 ray_origin_xz = {ray.origin[0], ray.origin[2]};
	double2 ray_dir_xz = {ray.direction[0], ray.direction[2]};

	double a = dot(ray_dir_xz, ray_dir_xz);
	double b = 2.0 * dot(ray_origin_xz, ray_dir_xz);
	double c = dot(ray_origin_xz, ray_origin_xz) - radius * radius;

	double discriminant = b * b - 4.0 * a * c;

	if (discriminant >= 0 && a > 0) {

		double t_one = (-b - sqrt(discriminant))/(2.0 * a);
		double t_two = (-b + sqrt(discriminant))/(2.0 * a);

		bool t_one_in_bounds = t_one >= t_min && t_one <= t_max;
		bool t_two_in_bounds = t_two >= t_min && t_two <= t_max;
		double3 position_one = ray.origin + t_one * ray.direction;
		double3 position_two = ray.origin + t_two * ray.direction;

		bool t_one_hit_finite_cylinder = fabs(position_one[1]) <= half_height;
		bool t_two_hit_finite_cylinder = fabs(position_two[1]) <= half_height;

		if (!t_one_in_bounds && !t_two_in_bounds) {
			return false;
		}

		else if (!t_one_in_bounds && t_two_in_bounds && t_two_hit_finite_cylinder) {
			hit->depth = t_two;
			hit->key_material = key_material;
			hit->position = position_two;
			hit->normal = -(double3{hit->position[0],0,hit->position[2]});
			hit->uv = double2{0.5 + atan2(hit->position[2], -hit->position[0])/(2.0 * PI), -(hit->position[1] - half_height)/(2.0 * half_height)};
			return true;
		}

		else if (t_one_in_bounds && !t_two_in_bounds && t_one_hit_finite_cylinder) {
			hit->depth = t_one;
			hit->key_material = key_material;
			hit->position = position_one;
			hit->normal = (double3{hit->position[0],0,hit->position[2]});
			hit->uv = double2{0.5 + atan2(hit->position[2], -hit->position[0])/(2.0 * PI), -(hit->position[1] - half_height)/(2.0 * half_height)};
			return true;
		}

		else if (t_one_in_bounds && t_two_in_bounds) {

			if (t_one_hit_finite_cylinder) {
				hit->depth = t_one;
				hit->key_material = key_material;
				hit->position = position_one;
				hit->normal = (double3{hit->position[0],0,hit->position[2]});
				hit->uv = double2{0.5 + atan2(hit->position[2], -hit->position[0])/(2.0 * PI), -(hit->position[1] - half_height)/(2.0 * half_height)};
				return true;
			}
			else if (t_two_hit_finite_cylinder) {	
				hit->depth = t_two;
				hit->key_material = key_material;
				hit->position = position_two;
				hit->normal = -(double3{hit->position[0],0,hit->position[2]});
				hit->uv = double2{0.5 + atan2(hit->position[2], -hit->position[0])/(2.0 * PI), -(hit->position[1] - half_height)/(2.0 * half_height)};
				return true;
			}
		}
	}
	return false;
}

AABB Cylinder::compute_aabb() {


	double3 p1 = mul(transform,{ radius,  half_height,  radius, 1}).xyz();
	double3 p2 = mul(transform,{ radius,  half_height, -radius, 1}).xyz();
	double3 p3 = mul(transform,{ radius, -half_height,  radius, 1}).xyz();
	double3 p4 = mul(transform,{ radius, -half_height, -radius, 1}).xyz();
	double3 p5 = mul(transform,{-radius,  half_height,  radius, 1}).xyz();
	double3 p6 = mul(transform,{-radius,  half_height, -radius, 1}).xyz();
	double3 p7 = mul(transform,{-radius, -half_height,  radius, 1}).xyz();
	double3 p8 = mul(transform,{-radius, -half_height, -radius, 1}).xyz();

	std::vector<double3> points = std::vector<double3>{p1, p2, p3, p4, p5, p6, p7, p8};

	return construct_aabb(points);

}


bool Mesh::local_intersect(Ray ray, double t_min, double t_max, Intersection* hit) {

	double t = t_max;
	bool no_intersection = true;
	Intersection curr_hit;

	for (int i = 0; i < triangles.size(); i++) {

		bool intersection = intersect_triangle(ray, t_min, t_max, triangles[i], &curr_hit);

		if (intersection && curr_hit.depth < t) {
			no_intersection = false;
			t = curr_hit.depth;
			*hit = curr_hit;
		}
	}

	if (no_intersection) {
		return false;
	}
	return true;
}

bool Mesh::intersect_triangle(Ray  ray, double t_min, double t_max, Triangle const tri, Intersection *hit)
{

	double3 const &p0 = positions[tri[0].pi]; 
	double3 const &p1 = positions[tri[1].pi]; 
	double3 const &p2 = positions[tri[2].pi]; 

	double3 edge_BC = p2 - p1;
	double3 edge_BA = p0 - p1;
	double3 BC_cross_BA = cross(edge_BC, edge_BA);
	double3 n = normalize(BC_cross_BA);

	double D = dot(n, p0);
	double d_dot_n = dot(ray.direction, n);
	
	if (d_dot_n != 0) {

		double t = (D - dot(ray.origin, n))/d_dot_n;

		if (t < t_min || t > t_max) {
			return false;
		}

		double3 position = ray.origin + t * ray.direction;

		double3 AB_cross_AT = cross(p1 - p0, position - p0);
		double3 BC_cross_BT = cross(p2 - p1, position - p1);
		double3 CA_cross_CT = cross(p0 - p2, position - p2);

		if (dot(AB_cross_AT, n) >= 0 &&
			dot(BC_cross_BT, n) >= 0 &&
			dot(CA_cross_CT, n) >= 0) {

			double a = dot(BC_cross_BT, n)/dot(BC_cross_BA, n);
			double b = dot(AB_cross_AT, n)/dot(BC_cross_BA, n);
			double c = dot(CA_cross_CT, n)/dot(BC_cross_BA, n);

			double3 normal;
			if (dot(ray.direction, n) < 0) {
				normal = a * normals[tri[0].ni] + b * normals[tri[2].ni] + c * normals[tri[1].ni];
			}
			else {
				normal = -a * normals[tri[0].ni] + b * normals[tri[2].ni] + c * normals[tri[1].ni];
			}

			normal = normalize(normal);

			hit->depth = t;
			hit->position = position;
			hit->normal = normal;
			hit->key_material = key_material;

			hit->uv = a * tex_coords[tri[0].ti] + b * tex_coords[tri[2].ti] + c * tex_coords[tri[1].ti];

			return true;
		}
	}

	return false;
}

AABB Mesh::compute_aabb() {

	std::vector<double3> points_local;
	points_local = positions;

	double3 new_point = positions[1] + EPSILON * 
					    normalize(cross((positions[0]-positions[1]),
					  				    (positions[1]-positions[2])));
	
	points_local.push_back(new_point);

	std::vector<double3> points_global;

	for (int i = 0; i < points_local.size(); i++) {
		points_global.push_back(mul(transform, {points_local[i],1}).xyz());
	}

	return construct_aabb(points_global);
}