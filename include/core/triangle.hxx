/*****************************************************************//**
 * \file   triangle.hxx
 * \brief  Triangle 
 *         edges_[0]  vertex(0) -> vertex(1)
 *         edges_[1]  vertex(1) -> vertex(2)
 *         edges_[2]  vertex(2) -> vertex(0)
 * \author zzm
 * \date   June 2026
 *********************************************************************/
#pragma once
#include "core/vec3.hxx"
#include <vector>
#include "core/vertex.hxx"
#include <memory>
#include <array>
#include <stdexcept>
class Edge;

class Triangle {
private:
	std::array<Vertex*, 3> vertices_{ nullptr, nullptr, nullptr };
	std::array<Edge*, 3> edges_{ nullptr, nullptr, nullptr };

public:
	int index = -1;
	Vec3 normal_;
	double area = 0.0;
	Triangle() = default;

	Triangle(int idx, Vertex* v0, Vertex* v1, Vertex* v2)
		: vertices_{ v0, v1, v2 }, index(idx) {
		computeNormalAndArea();
	}
	void computeNormalAndArea() {
		Vertex* v0 = vertices_[0];
		Vertex* v1 = vertices_[1];
		Vertex* v2 = vertices_[2];

		if (!v0 || !v1 || !v2) {
			normal_ = Vec3(0.0, 0.0, 0.0);
			area = 0.0;
			return;
		}

		Vec3 e1(
			v1->x - v0->x,
			v1->y - v0->y,
			v1->z - v0->z
		);

		Vec3 e2(
			v2->x - v0->x,
			v2->y - v0->y,
			v2->z - v0->z
		);

		normal_ = e1.cross(e2);

		double len = normal_.cachedLength();

		area = 0.5 * len;

		if (len < 1e-12) {
			normal_ = Vec3(0.0, 0.0, 0.0);
		}
		else {
			normal_ = Vec3(
				normal_.x / len,
				normal_.y / len,
				normal_.z / len
			);
		}
	}

	const Vec3& normal() const noexcept {
		return normal_;
	}
	Vertex* v0() const noexcept { return vertices_[0]; }
	Vertex* v1() const noexcept { return vertices_[1]; }
	Vertex* v2() const noexcept { return vertices_[2]; }

	void setEdge(int i, Edge* e) {
		if (i < 0 || i > 2) throw std::out_of_range("Triangle edge index out of range [0,2]");
		edges_[i] = e;
	}
	void setEdges(Edge* e0, Edge* e1, Edge* e2) {
		edges_[0] = e0;
		edges_[1] = e1;
		edges_[2] = e2;
	}
	Edge* edge(int i) const {
		if (i < 0 || i > 2) throw std::out_of_range("Triangle edge index out of range [0,2]");
		return edges_[i];
	}

	Vertex* vertex(int i) const {
		if (i < 0 || i > 2) throw std::out_of_range("Triangle vertex index out of range [0,2]");
		return vertices_[i];
	}

};
