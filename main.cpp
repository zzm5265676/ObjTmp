#include<iostream>
#include<Windows.h>
#include<fstream>
#include<string>
#include "utils.hxx"
#include "point.hxx"
#include "vec3.hxx"
#include "triangle.hxx"
int main() {

#if 0 
	std::string filePath = "D:/Code/c/test01/a.obj";
	std::ifstream input(filePath);
	if (input.is_open()) {
		std::cout << "open success" << std::endl;
	}
	std::string str;
	int i = 0;
	std::vector<Point*> points;
	std::vector<Triangle*> triangels;
	double x, y, z;
	int o, p, q;
	while (std::getline(input, str)) {
		char type[10];
		sscanf(str.c_str(), "%s %lf %lf %lf", type, &x, &y, &z);
		if (int(type[0]) == 118) {
			Point* point = new Point(x, y, z);
			points.push_back(point);
		}
		if (int(type[0]) == 102) {
			sscanf(str.c_str(), "%s %d %d %d", type, &o, &p, &q);
			Triangle* triangle = new Triangle(points[o - 1], points[p - 1], points[q - 1]);
			triangels.push_back(triangle);
		}
		//std::cout << int(type[0]) << "  x:" << x << "  y:" << y << "  z:" << z << std::endl;
	}
#endif
	std::cout << "hello world" << std::endl;
	Point* p1 = new Point(1, 0, 0);
	Point* p2 = new Point(0, 1, 0);
	Point* p3 = new Point(0, 0, 1);

	std::cout << p1 << std::endl;


	return 0;
}