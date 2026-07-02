#include "creation.hxx"
#include <cmath>
#include <stdexcept>

namespace mesh_creation {

// ===== Box =====

Mesh api_make_box(double width, double height, double depth) {
	double hw = width * 0.5;
	double hh = height * 0.5;
	double hd = depth * 0.5;
	return api_make_box(Vec3(-hw, -hh, -hd), Vec3(hw, hh, hd));
}

Mesh api_make_box(const Vec3& min, const Vec3& max) {
	Mesh mesh;

	// 8 vertices
	//   v5----v6
	//  /|    /|
	// v1----v2 |
	// |v4---|v7
	// |/    |/
	// v0----v3
	int v0 = mesh.addVertex(min.x, min.y, min.z)->index;
	int v1 = mesh.addVertex(min.x, max.y, min.z)->index;
	int v2 = mesh.addVertex(max.x, max.y, min.z)->index;
	int v3 = mesh.addVertex(max.x, min.y, min.z)->index;
	int v4 = mesh.addVertex(min.x, min.y, max.z)->index;
	int v5 = mesh.addVertex(min.x, max.y, max.z)->index;
	int v6 = mesh.addVertex(max.x, max.y, max.z)->index;
	int v7 = mesh.addVertex(max.x, min.y, max.z)->index;

	// 6 faces, 2 triangles each, CCW winding when viewed from outside
	// Front (-Z): outward normal = (0,0,-1)
	mesh.addTriangle(v0, v1, v2);
	mesh.addTriangle(v0, v2, v3);
	// Back (+Z): outward normal = (0,0,+1)
	mesh.addTriangle(v4, v7, v6);
	mesh.addTriangle(v4, v6, v5);
	// Left (-X): outward normal = (-1,0,0)
	mesh.addTriangle(v0, v4, v5);
	mesh.addTriangle(v0, v5, v1);
	// Right (+X): outward normal = (+1,0,0)
	mesh.addTriangle(v3, v2, v6);
	mesh.addTriangle(v3, v6, v7);
	// Bottom (-Y): outward normal = (0,-1,0)
	mesh.addTriangle(v0, v3, v7);
	mesh.addTriangle(v0, v7, v4);
	// Top (+Y): outward normal = (0,+1,0)
	mesh.addTriangle(v1, v5, v6);
	mesh.addTriangle(v1, v6, v2);

	return mesh;
}

// ===== Sphere (UV sphere) =====

Mesh api_make_sphere(double radius, int option) {
	if (option < 4) option = 4;
	int lonSegs = option;
	int latSegs = option / 2;
	if (latSegs < 2) latSegs = 2;

	Mesh mesh;
	const double PI = 3.14159265358979323846;

	// Top pole
	int topIdx = mesh.addVertex(0, radius, 0)->index;

	// Latitude rings (top to bottom, excluding poles)
	for (int i = 1; i < latSegs; ++i) {
		double theta = PI * i / latSegs;
		double sinT = std::sin(theta);
		double cosT = std::cos(theta);
		for (int j = 0; j < lonSegs; ++j) {
			double phi = 2.0 * PI * j / lonSegs;
			double x = radius * sinT * std::cos(phi);
			double y = radius * cosT;
			double z = radius * sinT * std::sin(phi);
			mesh.addVertex(x, y, z);
		}
	}

	// Bottom pole
	int botIdx = mesh.addVertex(0, -radius, 0)->index;

	// Top cap: CCW from outside
	for (int j = 0; j < lonSegs; ++j) {
		int next = (j + 1) % lonSegs;
		mesh.addTriangle(topIdx, 1 + next, 1 + j);
	}

	// Middle strips: CCW from outside
	for (int i = 0; i < latSegs - 2; ++i) {
		int row0 = 1 + i * lonSegs;
		int row1 = 1 + (i + 1) * lonSegs;
		for (int j = 0; j < lonSegs; ++j) {
			int next = (j + 1) % lonSegs;
			mesh.addTriangle(row0 + j, row1 + next, row1 + j);
			mesh.addTriangle(row0 + j, row0 + next, row1 + next);
		}
	}

	// Bottom cap: CCW from outside
	int lastRing = 1 + (latSegs - 2) * lonSegs;
	for (int j = 0; j < lonSegs; ++j) {
		int next = (j + 1) % lonSegs;
		mesh.addTriangle(lastRing + j, lastRing + next, botIdx);
	}

	return mesh;
}

// ===== Torus =====

Mesh api_make_torus(double outerRadius, double innerRadius, int option) {
	if (option < 4) option = 4;
	int majorSegs = option;
	int minorSegs = option / 2;
	if (minorSegs < 4) minorSegs = 4;

	double majorR = (outerRadius + innerRadius) * 0.5;
	double minorR = (outerRadius - innerRadius) * 0.5;
	if (minorR < 0) minorR = 0;

	Mesh mesh;
	const double PI = 3.14159265358979323846;

	// Generate vertices
	for (int i = 0; i < majorSegs; ++i) {
		double theta = 2.0 * PI * i / majorSegs;
		double cosT = std::cos(theta);
		double sinT = std::sin(theta);
		for (int j = 0; j < minorSegs; ++j) {
			double phi = 2.0 * PI * j / minorSegs;
			double cosP = std::cos(phi);
			double sinP = std::sin(phi);
			double x = (majorR + minorR * cosP) * cosT;
			double y = minorR * sinP;
			double z = (majorR + minorR * cosP) * sinT;
			mesh.addVertex(x, y, z);
		}
	}

	// Generate faces: CCW from outside
	for (int i = 0; i < majorSegs; ++i) {
		int nextI = (i + 1) % majorSegs;
		for (int j = 0; j < minorSegs; ++j) {
			int nextJ = (j + 1) % minorSegs;
			int v00 = i * minorSegs + j;
			int v10 = nextI * minorSegs + j;
			int v01 = i * minorSegs + nextJ;
			int v11 = nextI * minorSegs + nextJ;
			mesh.addTriangle(v00, v11, v10);
			mesh.addTriangle(v00, v01, v11);
		}
	}

	return mesh;
}

// ===== Prism / Frustum =====

Mesh api_make_prism(int sides, double height, double bottomRadius, double topRadius) {
	if (sides < 3) sides = 3;

	Mesh mesh;
	const double PI = 3.14159265358979323846;
	double halfH = height * 0.5;

	// Bottom ring
	int botStart = 0;
	for (int i = 0; i < sides; ++i) {
		double angle = 2.0 * PI * i / sides;
		mesh.addVertex(bottomRadius * std::cos(angle), -halfH, bottomRadius * std::sin(angle));
	}

	// Top ring
	int topStart = sides;
	for (int i = 0; i < sides; ++i) {
		double angle = 2.0 * PI * i / sides;
		mesh.addVertex(topRadius * std::cos(angle), halfH, topRadius * std::sin(angle));
	}

	// Bottom face (fan): normal should point -Y (downward)
	// CCW when viewed from below: reverse order
	for (int i = 1; i < sides - 1; ++i) {
		mesh.addTriangle(botStart, botStart + i, botStart + i + 1);
	}

	// Top face (fan): normal should point +Y (upward)
	// CCW when viewed from above
	for (int i = 1; i < sides - 1; ++i) {
		mesh.addTriangle(topStart, topStart + i + 1, topStart + i);
	}

	// Side faces: outward-facing
	for (int i = 0; i < sides; ++i) {
		int next = (i + 1) % sides;
		int b0 = botStart + i;
		int b1 = botStart + next;
		int t0 = topStart + i;
		int t1 = topStart + next;
		mesh.addTriangle(b0, t1, b1);
		mesh.addTriangle(b0, t0, t1);
	}

	return mesh;
}

// ===== Pyramid =====

Mesh api_make_pyramid(int sides, double bottomRadius, double height) {
	if (sides < 3) sides = 3;

	Mesh mesh;
	const double PI = 3.14159265358979323846;

	// Bottom ring
	int botStart = 0;
	for (int i = 0; i < sides; ++i) {
		double angle = 2.0 * PI * i / sides;
		mesh.addVertex(bottomRadius * std::cos(angle), 0, bottomRadius * std::sin(angle));
	}

	// Apex
	int apex = mesh.addVertex(0, height, 0)->index;

	// Bottom face (fan): normal -Y, CCW from below
	for (int i = 1; i < sides - 1; ++i) {
		mesh.addTriangle(botStart, botStart + i, botStart + i + 1);
	}

	// Side faces: outward-facing
	for (int i = 0; i < sides; ++i) {
		int next = (i + 1) % sides;
		mesh.addTriangle(botStart + i, apex, botStart + next);
	}

	return mesh;
}

// ===== Cone =====

Mesh api_make_cone(double bottomRadius, double height, int option) {
	return api_make_pyramid(option, bottomRadius, height);
}

// ===== Frustum / Truncated Cone =====

Mesh api_make_frustum(double bottomRadius, double topRadius, double height, int option) {
	return api_make_prism(option, height, bottomRadius, topRadius);
}

} // namespace mesh_creation
