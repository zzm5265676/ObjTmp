#pragma once
#include <string>

namespace config {
#ifdef OBJTMP_PROJECT_ROOT
	const std::string ROOT_DIR = std::string(OBJTMP_PROJECT_ROOT) + "/";
#else
	const std::string ROOT_DIR = "";
#endif
	const std::string DATA_DIR = ROOT_DIR + "data/";
	const std::string INPUT_DIR = DATA_DIR + "input/";
	const std::string OUTPUT_DIR = DATA_DIR + "output/";
}
