#include "raytracer.h"

void Raytracer::render(const Scene& scene, Frame* output)
{       

    double *z_buffer = new double[scene.resolution[0] * scene.resolution[1]];
    for(int i = 0; i < scene.resolution[0] * scene.resolution[1]; i++) {
        z_buffer[i] = scene.camera.z_far;
    }

	double3 n = scene.camera.position - scene.camera.center;
	double distance = length(n);

			n = normalize(n);
	double3 u = normalize(cross(scene.camera.up, n));
	double3 v = cross(n, u);


	double half_height = distance * tan(deg2rad(scene.camera.fovy)/2.0);
	double half_width = scene.camera.aspect * half_height;

	double3 bottom_left = scene.camera.center - half_width * u - half_height * v;

	double3 pixel_u = ((half_width * 2.0)/scene.resolution[0]) * u;
	double3 pixel_v = ((half_height * 2.0)/scene.resolution[1]) * v;

    for(int y = 0; y < scene.resolution[1]; y++) {
		if (y % 40){
			std::cout << "\rScanlines completed: " << y << "/" << scene.resolution[1] << '\r';
		}

        for(int x = 0; x < scene.resolution[0]; x++) {

			int avg_z_depth = 0;
			double3 avg_ray_color{0,0,0};
			
			for(int iray = 0; iray < scene.samples_per_pixel; iray++) {

				Ray ray;
				int ray_depth = 0;
				double3 ray_color = {0,0,0};
				double z_depth = scene.camera.z_far;

				double jitter_x = rand()/((double) RAND_MAX) * 2 * scene.jitter_radius - scene.jitter_radius;
				double jitter_y = rand()/((double) RAND_MAX) * 2 * scene.jitter_radius - scene.jitter_radius;

				double3 pixel_center = bottom_left + (x + 0.5 + jitter_x) * pixel_u + (y + 0.5 + jitter_y) * pixel_v;

				ray.origin = scene.camera.position;
				ray.direction = pixel_center - scene.camera.position;

				trace(scene, ray, ray_depth, &ray_color, &z_depth);

				avg_z_depth += z_depth;
				avg_ray_color += ray_color;

			}
			
			avg_z_depth = avg_z_depth / scene.samples_per_pixel;
			avg_ray_color = avg_ray_color / scene.samples_per_pixel;

			if (avg_z_depth >= scene.camera.z_near && avg_z_depth <= scene.camera.z_far && 
				avg_z_depth < z_buffer[x + y*scene.resolution[0]]) {
				z_buffer[x + y*scene.resolution[0]] = avg_z_depth;

				output->set_color_pixel(x, y, avg_ray_color);
				output->set_depth_pixel(x, y, (avg_z_depth - scene.camera.z_near) / (scene.camera.z_far-scene.camera.z_near));
			}
        }
    }

    delete[] z_buffer;
}

void Raytracer::trace(const Scene& scene, Ray ray, int ray_depth, double3* out_color, double* out_z_depth) {
	Intersection hit;
	
	if(scene.container->intersect(ray,EPSILON,DBL_MAX,&hit)) {		
		Material& material = ResourceManager::Instance()->materials[hit.key_material];

		double3 color_reflection = {0, 0, 0};
		if (ray_depth < scene.max_ray_depth && material.k_reflection > 0) {

			Ray reflected_ray;
			reflected_ray.origin = hit.position; 
 			reflected_ray.direction = ray.direction - 2.0 * dot(ray.direction, hit.normal) * hit.normal;

			trace(scene, reflected_ray, ray_depth + 1, &color_reflection, out_z_depth);		

		}

		double3 color_refraction = {0, 0, 0};
		if (ray_depth < scene.max_ray_depth && material.k_refraction > 0) {

			Ray refracted_ray;
			refracted_ray.origin = hit.position;
			
			double eta = 1.0/material.refractive_index;

			double nv = dot(hit.normal, normalize(-ray.direction));

			double root_term = 1.0 - eta * eta * (1.0 - nv * nv);

			if (root_term >= 0) {

				refracted_ray.direction = normalize(hit.normal * (eta * nv - sqrt(root_term)) + eta * normalize(ray.direction));

				trace(scene, refracted_ray, ray_depth + 1, &color_refraction, out_z_depth);	
			}	
		}

		*out_color = shade(scene, hit) + color_reflection * material.k_reflection + color_refraction * material.k_refraction;

		if (ray_depth == 0) {
			*out_z_depth = length(hit.position - scene.camera.position);	
		}
	}
}


double3 Raytracer::shade(const Scene& scene, Intersection hit)
{
	Material& material = ResourceManager::Instance()->materials[hit.key_material];
	double3 color = {0,0,0};

	if (material.texture_albedo.width() == 0 && material.texture_albedo.height() == 0) {
		color = material.color_albedo;
	}

	else {
		unsigned char red = 0;
		unsigned char green = 0;
		unsigned char blue = 0;
		material.texture_albedo.get_pixel((int)(hit.uv[0] * material.texture_albedo.width()),
										(int)(hit.uv[1] * material.texture_albedo.height()),
										 red, green, blue);

		color = {red/255.0, green/255.0, blue/255.0};
	}
	
	double3 ambient_light = scene.ambient_light * material.k_ambient * color;

	double3 diffuse_light = {0,0,0};
	double3 specular_light = {0,0,0};

	for (int i = 0; i < scene.lights.size(); i++) {

		SphericalLight light = scene.lights[i];

		Ray hit_to_light;
		Intersection blocking_hit;

		if (light.radius == 0) {

			hit_to_light.origin = hit.position;
			hit_to_light.direction = light.position - hit_to_light.origin;

			if (!scene.container->intersect(hit_to_light,EPSILON,1.0,&blocking_hit)) {

				diffuse_light += (light.emission * material.k_diffuse * 
								std::max(dot(hit.normal, normalize(hit_to_light.direction)),0.0)/pow(length(hit_to_light.direction),2) * 
								color);

				double3 e = normalize(scene.camera.position - hit.position);
				double3 h = normalize(normalize(hit_to_light.direction) + e);

				specular_light += ((material.metallic  * color + (1 - material.metallic)) *
								material.k_specular * light.emission * 
								pow(dot(hit.normal, h), material.shininess) / pow(length(hit_to_light.direction), 2));
			}
		}

		else {

			double3 n = normalize(hit.position - light.position);
			double3 u = normalize(cross(scene.camera.up, n));
			double3 v = cross(n, u);
			
			int nb_samples = 10;

			int nb_hit = 0;

			for (int i = 0; i < nb_samples; i++) {

				double2 random = random_in_unit_disk();
				double3 light_position = light.position + random[0] * u * light.radius + random[1] * v * light.radius;

				hit_to_light.origin = hit.position;
				hit_to_light.direction = light_position - hit_to_light.origin;
				
				if (!scene.container->intersect(hit_to_light,EPSILON,1.0,&blocking_hit)) {
					nb_hit++;
				}
			}


			hit_to_light.origin = hit.position;
			hit_to_light.direction = light.position - hit_to_light.origin;

			diffuse_light += (light.emission * material.k_diffuse * 
							std::max(dot(hit.normal, normalize(hit_to_light.direction)),0.0)/pow(length(hit_to_light.direction),2) * 
							color) *
							 (((double) nb_hit)/((double) nb_samples));

			double3 e = normalize(scene.camera.position - hit.position);
			double3 h = normalize(normalize(hit_to_light.direction) + e);

			specular_light += ((material.metallic  * color + (1 - material.metallic)) *
								material.k_specular * light.emission * 
								pow(dot(hit.normal, h), material.shininess) / pow(length(hit_to_light.direction), 2)) *
								 (((double) nb_hit)/((double) nb_samples));
		}
	}

	color = ambient_light + diffuse_light + specular_light;

	return color;
}