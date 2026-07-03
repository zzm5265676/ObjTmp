#pragma once
#include "mesh/mesh.hxx"
#include <cmath>

namespace mesh_creation {

	// Box from width/height/depth (centered at origin)
	Mesh api_make_box(double width, double height, double depth);

	// Box from two corner points
	Mesh api_make_box(const Vec3& min, const Vec3& max);

	// Sphere (UV sphere, option = longitude segments, default 32)
	Mesh api_make_sphere(double radius, int option = 32);

	// Torus (option = tube segments, default 32)
	Mesh api_make_torus(double outerRadius, double innerRadius, int option = 32);

	// Prism / Frustum (polygon base, option not used)
	// sides = number of polygon edges, bottomRadius/topRadius = base radii
	Mesh api_make_prism(int sides, double height, double bottomRadius, double topRadius);

	// Pyramid (polygon base, top degenerates to a point)
	Mesh api_make_pyramid(int sides, double bottomRadius, double height);

	// Cone (discretized pyramid, option controls circle smoothness, default 32)
	Mesh api_make_cone(double bottomRadius, double height, int option = 32);

	// Frustum / truncated cone (option controls circle smoothness, default 32)
	Mesh api_make_frustum(double bottomRadius, double topRadius, double height, int option = 32);

} // namespace mesh_creation
