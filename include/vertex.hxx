#pragma once
#include "point.hxx"
#include "vec3.hxx"
#include <vector>
#include <algorithm>

class Edge;
class Triangle;

class Vertex : public Point {
private:
	std::vector<Vertex*> neivts;
	std::vector<Edge*> neiedges;
	std::vector<Triangle*> neitris;

public:
	int index = -1;

	Vertex() = default;
	Vertex(int idx, double x_, double y_, double z_)
		: Point(x_, y_, z_), index(idx) {
	}

	Vertex(int idx, Point* pt)
		: Point(*pt), index(idx) {
	}

	Vertex(Vertex* vt)
		: Point(*vt) {
	}

	void initIndex(int idx) {
		index = idx;
	}

	bool addNeiVertex(Vertex* vt) {
		if (!vt || vt == this) return false;
		for (Vertex* v : neivts) {
			if (v == vt) return false;
		}
		neivts.push_back(vt);
		return true;
	}

	bool addNeiEdge(Edge* edge) {
		if (!edge) return false;
		for (Edge* e : neiedges) {
			if (e == edge) return false;
		}
		neiedges.push_back(edge);
		return true;
	}

	bool addNeiTri(Triangle* tri) {
		if (!tri) return false;
		for (Triangle* t : neitris) {
			if (t == tri) return false;
		}
		neitris.push_back(tri);
		return true;
	}

	const std::vector<Vertex*>& getNeiVertics() const noexcept {
		return neivts;
	}

	const std::vector<Edge*>& getNeiEdges() const noexcept {
		return neiedges;
	}

	const std::vector<Triangle*>& getNeiTriangles() const noexcept {
		return neitris;
	}
};
