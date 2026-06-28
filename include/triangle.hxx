/*****************************************************************//**
 * \file   triangle.hxx
 * \brief  Triangle 是一个有方向的三角面。
 *         edges_[0] 应该对应 vertex(0) -> vertex(1)
 *         edges_[1] 应该对应 vertex(1) -> vertex(2)
 *         edges_[2] 应该对应 vertex(2) -> vertex(0)
 * \author zzm
 * \date   June 2026
 *********************************************************************/
#pragma once
#include "vec3.hxx"
#include <vector>
#include "vertex.hxx"
#include <memory>
#include <array>
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

		double len = normal_.length();

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
		edges_[i] = e;
	}
	void setEdges(Edge* e0, Edge* e1, Edge* e2) {
		if (!edges_[0])setEdge(0, e0);
		if (!edges_[1])setEdge(1, e1);
		if (!edges_[2])setEdge(2, e2);
	}
	Edge* edge(int i) const noexcept {
		return edges_[i];
	}

	Vertex* vertex(int i) const noexcept {
		return vertices_[i];
	}

};
