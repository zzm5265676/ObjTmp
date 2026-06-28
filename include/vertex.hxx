#pragma once
#include "point.hxx"
#include "vec3.hxx"
#include <unordered_set>

class Edge;
class Triangle;

class Vertex : public Point {
private:
	std::unordered_set<Vertex*> neivts;
	std::unordered_set<Edge*> neiedges;
	std::unordered_set<Triangle*> neitris;
public:
	int index = -1;

	Vertex() = default;
	Vertex(int idx, double x_, double y_, double z_) {
		this->index = idx;
		this->x = x_;
		this->y = y_;
		this->z = z_;
	}
	Vertex(int idx, Point* pt) {
		this->y = pt->y;
		this->z = pt->z;
		this->x = pt->x;
		this->index = idx;
	}
	Vertex(Vertex* vt) {
		this->x = vt->x;
		this->y = vt->y;
		this->z = vt->z;
	}
	void initIndex(int idx) {
		index = idx;
	}

	bool addNeiVertex(Vertex* vt) {
		if (!vt) return false;
		return neivts.insert(vt).second;
	}

	bool addNeiEdge(Edge* edge) {
		if (!edge) return false;
		return neiedges.insert(edge).second;
	}

	bool addNeiTri(Triangle* tri) {
		if (!tri) return false;
		return neitris.insert(tri).second;
	}

	struct EdgeEqualByVertexIndex {
		bool operator()(const Edge* a, const Edge* b) const noexcept;
	};

	const std::unordered_set<Vertex*>& getNeiVertics() const noexcept {
		return neivts;
	}

	const std::unordered_set<Edge*>& getNeiEdges() const noexcept {
		return neiedges;
	}

	const std::unordered_set<Triangle*>& getNeiTriangles() const noexcept {
		return neitris;
	}
};
