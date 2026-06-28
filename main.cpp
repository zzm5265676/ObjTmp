#include<iostream>
#include<Windows.h>
#include<fstream>
#include<string>
//#include "utils.hxx"
#include "mesh.hxx"

int main() {
	std::string filePath = "D:/Code/ObjTmp/a.obj";
	std::string outputPath = "D:/Code/ObjTmp/exported.obj";
	std::string basePath = "D:/Code/ObjTmp/";
	Mesh mesh(filePath);
	try {
		std::cout << "load success" << std::endl;
		std::cout << "vertex count: " << mesh.vertexCount() << std::endl;
		std::cout << "edge count: " << mesh.edgeCount() << std::endl;
		std::cout << "triangle count: " << mesh.triangleCount() << std::endl;
	}
	catch (const std::exception& e) {
		std::cout << e.what() << std::endl;
		return 1;
	}
	std::vector<std::vector<Vertex*>> components = mesh.connectedVertexComponents();
	for (std::size_t i = 0; i < components.size(); ++i) {
		auto tris = mesh.collectTriangleFromVertexComponent(components[i]);

		std::cout << "component " << i
			<< " vertex count = " << components[i].size()
			<< ", triangle count = " << tris.size()
			<< std::endl;
	}
	std::vector<Mesh> meshs = mesh.splitComponents();
	int index = 0;
	for (std::vector<Mesh>::iterator it = meshs.begin(); it != meshs.end(); it++) {
		it->exportObj(basePath + std::to_string(index) + ".obj");
		index++;
	}



	std::vector<std::vector<Triangle*>> tricomps = mesh.connectedTriangleComponents();
	mesh.exportObj(outputPath);



	return 0;
}