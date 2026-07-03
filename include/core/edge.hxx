/*****************************************************************//**
 * \file   edge.hxx
 * \brief  
 *         Edge(v0, v1)  v0 -> v1
 *         oppositeEdge_  v1 -> v0
 *         triangles_  Triangle
 * \author zzm
 * \date   June 2026
 *********************************************************************/
#pragma once
#include "core/vertex.hxx"
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
		// 
		if (this->v0_ != eg->v1_ || this->v1_ != eg->v0_) {
			return false;
		}
		//  opposite
		if (oppositeEdge_ == eg && eg->oppositeEdge_ == this) {
			return true;
		}
		//  this  opposite
		if (oppositeEdge_ != nullptr && oppositeEdge_ != eg) {
			return false;
		}

		//  eg  opposite
		if (eg->oppositeEdge_ != nullptr && eg->oppositeEdge_ != this) {
			return false;
		}

		// 
		oppositeEdge_ = eg;
		eg->oppositeEdge_ = this;

		return true;
	}
	const std::unordered_set<Triangle*>& triangles() const noexcept {
		return triangles_;
	}
};
