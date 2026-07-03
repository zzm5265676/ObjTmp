#pragma once
#include "mesh/mesh.hxx"
#include <fstream>
#include <string>

namespace mesh_io {

// Export mesh to PLY format (ASCII)
inline bool exportPly(const Mesh& mesh, const std::string& filePath) {
	std::ofstream out(filePath);
	if (!out.is_open()) return false;

	int nVerts = static_cast<int>(mesh.vertexCount());
	int nFaces = static_cast<int>(mesh.triangleCount());

	// Header
	out << "ply\n";
	out << "format ascii 1.0\n";
	out << "element vertex " << nVerts << "\n";
	out << "property float x\n";
	out << "property float y\n";
	out << "property float z\n";
	out << "property float nx\n";
	out << "property float ny\n";
	out << "property float nz\n";
	out << "element face " << nFaces << "\n";
	out << "property list uchar int vertex_indices\n";
	out << "end_header\n";

	// Compute vertex normals (area-weighted)
	auto normals = mesh.computeVertexNormals(NormalComputer::VertexNormalMethod::AreaWeighted);

	// Vertices
	for (int i = 0; i < nVerts; ++i) {
		const Vertex* v = mesh.findByIndex(i);
		if (!v) continue;
		out << v->x << " " << v->y << " " << v->z << " "
		    << normals[i].x << " " << normals[i].y << " " << normals[i].z << "\n";
	}

	// Faces
	for (int i = 0; i < nFaces; ++i) {
		const Triangle* tri = mesh.triangle(i);
		if (!tri) continue;
		out << "3 " << tri->vertex(0)->index << " "
		    << tri->vertex(1)->index << " " << tri->vertex(2)->index << "\n";
	}

	return true;
}

// Export mesh to STL format (ASCII)
inline bool exportStl(const Mesh& mesh, const std::string& filePath) {
	std::ofstream out(filePath);
	if (!out.is_open()) return false;

	out << "solid mesh\n";

	for (std::size_t i = 0; i < mesh.triangleCount(); ++i) {
		const Triangle* tri = mesh.triangle(static_cast<int>(i));
		if (!tri) continue;

		const Vec3& n = tri->normal();
		out << "  facet normal " << n.x << " " << n.y << " " << n.z << "\n";
		out << "    outer loop\n";

		for (int j = 0; j < 3; ++j) {
			const Vertex* v = tri->vertex(j);
			if (v) {
				out << "      vertex " << v->x << " " << v->y << " " << v->z << "\n";
			}
		}

		out << "    endloop\n";
		out << "  endfacet\n";
	}

	out << "endsolid mesh\n";
	return true;
}

} // namespace mesh_io
