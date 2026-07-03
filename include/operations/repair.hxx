#pragma once
#include "mesh/mesh.hxx"
#include "geometry/geometry_utils.hxx"
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <cmath>

namespace mesh_repair {

struct RepairOptions {
	double mergeVertexEps = 1e-6;
	double degenerateEps = 1e-12;
	bool fixOrientation = true;
	bool removeDegenerate = true;
	bool removeDuplicates = true;
	bool removeIsolated = true;
	bool fillHoles = true;
};

struct RepairReport {
	int mergedVertices = 0;
	int removedDegenerate = 0;
	int removedDuplicates = 0;
	int removedIsolated = 0;
	int flippedFaces = 0;
	int filledHoles = 0;
};

// Forward declarations
int mergeCloseVertices(Mesh& mesh, double eps);
int removeDegenerateTriangles(Mesh& mesh, double eps);
int removeDuplicateTriangles(Mesh& mesh);
int removeIsolatedVertices(Mesh& mesh);
int fixOrientation(Mesh& mesh);
int fillHoles(Mesh& mesh);

// One-click repair
inline RepairReport repair(Mesh& mesh, const RepairOptions& opts = {}) {
	RepairReport report;
	report.mergedVertices = mergeCloseVertices(mesh, opts.mergeVertexEps);
	if (opts.removeDegenerate)
		report.removedDegenerate = removeDegenerateTriangles(mesh, opts.degenerateEps);
	if (opts.removeDuplicates)
		report.removedDuplicates = removeDuplicateTriangles(mesh);
	if (opts.fixOrientation)
		report.flippedFaces = fixOrientation(mesh);
	if (opts.removeIsolated)
		report.removedIsolated = removeIsolatedVertices(mesh);
	if (opts.fillHoles)
		report.filledHoles = fillHoles(mesh);
	return report;
}

} // namespace mesh_repair
