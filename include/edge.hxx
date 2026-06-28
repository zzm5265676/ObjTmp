#pragma once
#include "vertex.hxx"
#include <unordered_set>

class Triangle;

class Edge {
private:
	// Directed edge endpoints.
	Vertex* v0_ = nullptr;
	Vertex* v1_ = nullptr;

	std::unordered_set<Triangle*> triangles_;

	Edge* oppositeEdge_ = nullptr;
public:
	int index = -1;
	Edge() = default;

	// Directed edge constructor.
	Edge(int idx, Vertex* v0, Vertex* v1) :v0_(v0), v1_(v1), index(idx) {

	}
	// Canonically ordered edge constructor.
	Edge(Vertex* v0, Vertex* v1)
		: Edge(-1,
			v0->index <= v1->index ? v0 : v1,
			v0->index <= v1->index ? v1 : v0) {
	}

	bool addTriangle(Triangle* tri) {
		if (!tri) return false;
		return triangles_.insert(tri).second;
	}

	Vertex* v0() const noexcept { return v0_; }
	Vertex* v1() const noexcept { return v1_; }
	Vertex* from() const noexcept { return v0_; }
	Vertex* to() const noexcept { return v1_; }
	Edge* opposite() const noexcept { return oppositeEdge_; }

	bool setOpposite(Edge* eg) {
		if (!eg) return false;
		if (eg == this) return false;
		// 检查两条边是否确实方向相反
		if (this->v0_ != eg->v1_ || this->v1_ != eg->v0_) {
			return false;
		}
		// 如果已经设置成这个 opposite，直接认为成功
		if (oppositeEdge_ == eg && eg->oppositeEdge_ == this) {
			return true;
		}
		// 如果 this 已经有别的 opposite，不允许覆盖
		if (oppositeEdge_ != nullptr && oppositeEdge_ != eg) {
			return false;
		}

		// 如果 eg 已经有别的 opposite，不允许覆盖
		if (eg->oppositeEdge_ != nullptr && eg->oppositeEdge_ != this) {
			return false;
		}

		// 建立双向关系
		oppositeEdge_ = eg;
		eg->oppositeEdge_ = this;

		return true;
	}
	const std::unordered_set<Triangle*>& triangles() const noexcept {
		return triangles_;
	}
};
