#include "config.hxx"
#include "mesh/mesh.hxx"

#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

int main(int argc, char** argv) {
	std::string inputPath = argc > 1 ? argv[1] : config::INPUT_DIR + "jiaban.obj";
	bool exportComponents = true;
	std::string outputBase = config::OUTPUT_DIR;
	int toleranceArg = 3;

	if (argc > 2) {
		std::string secondArg = argv[2];
		if (secondArg == "--count-only") {
			exportComponents = false;
			toleranceArg = 3;
		}
		else {
			outputBase = secondArg;
			toleranceArg = 3;
		}
	}

	try {
		double connectTolerance = argc > toleranceArg ? std::stod(argv[toleranceArg]) : 1.0e-7;
		if (connectTolerance < 0.0) {
			throw std::runtime_error("connectTolerance must be non-negative");
		}

		std::cout << "loading: " << inputPath << "\n";
		std::cout << "connect tolerance: " << connectTolerance << "\n";
		Mesh mesh(inputPath, connectTolerance);

		std::cout << "vertices: " << mesh.vertexCount() << "\n";
		std::cout << "directed edges: " << mesh.edgeCount() << "\n";
		std::cout << "triangles: " << mesh.triangleCount() << "\n";
		if (!mesh.parseErrors().empty()) {
			std::cout << "parse warnings: " << mesh.parseErrors().size() << "\n";
		}

		std::cout << "splitting components...\n";
		std::vector<Mesh> components = mesh.splitComponents();
		std::cout << "components: " << components.size() << "\n";

		if (!exportComponents) {
			return 0;
		}

		for (std::size_t i = 0; i < components.size(); ++i) {
			std::string outputPath = outputBase + "component_" + std::to_string(i) + ".obj";
			if (!components[i].exportObj(outputPath)) {
				std::cerr << "failed to export: " << outputPath << "\n";
				return 1;
			}

			std::cout << "component " << i
				<< ": vertices=" << components[i].vertexCount()
				<< ", directed_edges=" << components[i].edgeCount()
				<< ", triangles=" << components[i].triangleCount()
				<< ", output=" << outputPath << "\n";
		}
	}
	catch (const std::exception& e) {
		std::cerr << "error: " << e.what() << "\n";
		return 1;
	}

	return 0;
}
