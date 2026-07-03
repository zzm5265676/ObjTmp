#pragma once
#include "mesh/mesh.hxx"
#include "geometry/geometry_utils.hxx"
#include <vector>
#include <queue>
#include <cmath>

namespace mesh_decimate {

struct Quadric {
	double m[10];

	Quadric() { for (int i = 0; i < 10; ++i) m[i] = 0; }

	static Quadric fromPlane(double a, double b, double c, double d) {
		Quadric q;
		q.m[0] = a*a; q.m[1] = a*b; q.m[2] = a*c; q.m[3] = a*d;
		q.m[4] = b*b; q.m[5] = b*c; q.m[6] = b*d;
		q.m[7] = c*c; q.m[8] = c*d;
		q.m[9] = d*d;
		return q;
	}

	static Quadric fromTriangle(const Triangle* tri) {
		if (!tri) return Quadric();
		const Vec3& n = tri->normal();
		Vertex* v0 = tri->vertex(0);
		if (!v0) return Quadric();
		double d = -(n.x * v0->x + n.y * v0->y + n.z * v0->z);
		return fromPlane(n.x, n.y, n.z, d);
	}

	Quadric operator+(const Quadric& rhs) const {
		Quadric r;
		for (int i = 0; i < 10; ++i) r.m[i] = m[i] + rhs.m[i];
		return r;
	}

	Quadric& operator+=(const Quadric& rhs) {
		for (int i = 0; i < 10; ++i) m[i] += rhs.m[i];
		return *this;
	}

	double evaluate(double x, double y, double z) const {
		return m[0]*x*x + 2*m[1]*x*y + 2*m[2]*x*z + 2*m[3]*x
		     + m[4]*y*y + 2*m[5]*y*z + 2*m[6]*y
		     + m[7]*z*z + 2*m[8]*z
		     + m[9];
	}

	bool optimalPosition(Vec3& pos) const {
		double a00 = m[0], a01 = m[1], a02 = m[2];
		double a11 = m[4], a12 = m[5];
		double a22 = m[7];
		double b0 = m[3], b1 = m[6], b2 = m[8];

		double det = a00 * (a11 * a22 - a12 * a12)
		           - a01 * (a01 * a22 - a12 * a02)
		           + a02 * (a01 * a12 - a11 * a02);

		if (std::abs(det) < 1e-15) return false;

		double invDet = 1.0 / det;
		pos.x = invDet * (b0 * (a12*a12 - a11*a22) + b1 * (a01*a22 - a02*a12) + b2 * (a02*a11 - a01*a12));
		pos.y = invDet * (b0 * (a02*a12 - a01*a22) + b1 * (a00*a22 - a02*a02) + b2 * (a01*a02 - a00*a12));
		pos.z = invDet * (b0 * (a01*a12 - a02*a11) + b1 * (a01*a02 - a00*a12) + b2 * (a00*a11 - a01*a01));
		return true;
	}
};

// QEM decimation
// targetRatio: fraction of triangles to keep (0.5 = reduce by 50%)
inline void decimateQEM(Mesh& mesh, double targetRatio = 0.5) {
	if (targetRatio <= 0 || targetRatio >= 1.0) return;

	int targetTriCount = static_cast<int>(mesh.triangleCount() * targetRatio);
	if (targetTriCount < 4) targetTriCount = 4;
	int maxCollapses = static_cast<int>(mesh.vertexCount()) - targetTriCount / 2;
	if (maxCollapses < 0) maxCollapses = 0;

	// Step 1: Compute initial quadrics
	int nVerts = static_cast<int>(mesh.vertexCount());
	std::vector<Quadric> vertexQuadrics(nVerts);
	for (std::size_t i = 0; i < mesh.triangleCount(); ++i) {
		const Triangle* tri = mesh.triangle(static_cast<int>(i));
		if (!tri) continue;
		Quadric q = Quadric::fromTriangle(tri);
		for (int j = 0; j < 3; ++j) {
			int idx = tri->vertex(j)->index;
			vertexQuadrics[idx] += q;
		}
	}

	// Step 2: Build edge queue
	struct EdgeCollapse {
		double error;
		Vec3 optimalPos;
		int v1, v2;
		bool operator>(const EdgeCollapse& o) const { return error > o.error; }
	};

	auto computeCollapse = [&](int v1idx, int v2idx) -> EdgeCollapse {
		EdgeCollapse ec;
		ec.v1 = v1idx;
		ec.v2 = v2idx;
		Quadric q = vertexQuadrics[v1idx] + vertexQuadrics[v2idx];

		Vec3 optPos;
		if (q.optimalPosition(optPos)) {
			ec.optimalPos = optPos;
			ec.error = q.evaluate(optPos.x, optPos.y, optPos.z);
		} else {
			const Vertex* va = mesh.findByIndex(v1idx);
			const Vertex* vb = mesh.findByIndex(v2idx);
			if (va && vb) {
				ec.optimalPos = Vec3((va->x + vb->x)*0.5, (va->y + vb->y)*0.5, (va->z + vb->z)*0.5);
				ec.error = q.evaluate(ec.optimalPos.x, ec.optimalPos.y, ec.optimalPos.z);
			} else {
				ec.error = 1e30;
			}
		}
		return ec;
	};

	std::priority_queue<EdgeCollapse, std::vector<EdgeCollapse>, std::greater<EdgeCollapse>> edgeQueue;
	for (const auto& entry : mesh.collectUndirectedEdges()) {
		edgeQueue.push(computeCollapse(entry.first.a, entry.first.b));
	}

	// Step 3: Collapse edges
	std::vector<bool> removed(nVerts, false);
	std::vector<int> mergedTo(nVerts);
	for (int i = 0; i < nVerts; ++i) mergedTo[i] = i;

	int collapses = 0;
	while (collapses < maxCollapses && !edgeQueue.empty()) {
		EdgeCollapse ec = edgeQueue.top();
		edgeQueue.pop();

		if (removed[ec.v1] || removed[ec.v2]) continue;

		// Check if collapse would create degenerate triangles
		// (triangles that have both v1 and v2)
		int sharedTris = 0;
		for (const Triangle* tri : mesh.findByIndex(ec.v2)->getNeiTriangles()) {
			if (!tri) continue;
			bool hasV1 = false, hasV2 = false;
			for (int j = 0; j < 3; ++j) {
				if (tri->vertex(j)->index == ec.v1) hasV1 = true;
				if (tri->vertex(j)->index == ec.v2) hasV2 = true;
			}
			if (hasV1 && hasV2) sharedTris++;
		}

		// Collapse
		Vertex* v1 = mesh.findByIndex(ec.v1);
		Vertex* v2 = mesh.findByIndex(ec.v2);
		if (!v1 || !v2) continue;

		v1->x = ec.optimalPos.x;
		v1->y = ec.optimalPos.y;
		v1->z = ec.optimalPos.z;
		vertexQuadrics[ec.v1] = vertexQuadrics[ec.v1] + vertexQuadrics[ec.v2];

		removed[ec.v2] = true;
		mergedTo[ec.v2] = ec.v1;
		collapses++;

		// Add new edges from v1 to v2's other neighbors
		for (const Vertex* neighbor : v2->getNeiVertics()) {
			if (!neighbor || neighbor->index == ec.v1 || removed[neighbor->index]) continue;
			edgeQueue.push(computeCollapse(ec.v1, neighbor->index));
		}
	}

	// Step 4: Rebuild mesh
	Mesh result;
	std::vector<int> oldToNew(nVerts, -1);

	auto findRoot = [&](int v) -> int {
		while (removed[v]) v = mergedTo[v];
		return v;
	};

	for (int i = 0; i < nVerts; ++i) {
		if (removed[i]) continue;
		const Vertex* v = mesh.findByIndex(i);
		if (v) oldToNew[i] = result.addVertex(v->x, v->y, v->z)->index;
	}

	for (std::size_t i = 0; i < mesh.triangleCount(); ++i) {
		const Triangle* tri = mesh.triangle(static_cast<int>(i));
		if (!tri) continue;

		int v0 = findRoot(tri->vertex(0)->index);
		int v1 = findRoot(tri->vertex(1)->index);
		int v2 = findRoot(tri->vertex(2)->index);

		int nv0 = oldToNew[v0];
		int nv1 = oldToNew[v1];
		int nv2 = oldToNew[v2];

		if (nv0 < 0 || nv1 < 0 || nv2 < 0) continue;
		if (nv0 == nv1 || nv1 == nv2 || nv2 == nv0) continue;

		result.addTriangle(nv0, nv1, nv2);
	}

	mesh = std::move(result);
}

} // namespace mesh_decimate
