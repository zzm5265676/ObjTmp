#pragma once
#include "point.hxx"
#include "vec3.hxx"
#include <unordered_set>
#include "triangle.hxx"
class Vertex :Point {
private:
	std::vector<Vertex*> neighbours;
	std::vector<Triangle*> triangles;
public:
};
