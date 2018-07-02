#include <iostream>
#include <fstream>
#include <sstream>
#include <limits>
#include "zonotope.h"

int myprintf(const char *fmt, ...)
{
    return 0;
}

int main(int argc, char *argv[]) {
    std::cout.precision(std::numeric_limits<double>::max_digits10);

    if (argc != 3) {
        std::cerr << "Invalid arguments" << std::endl;
        return 1;
    }

	unsigned int dim;
    std::vector<Eigen::VectorXd> star, vertices;
	std::vector<int> edges;

    star.clear();
    vertices.clear();
    edges.clear();

    if ((dim = readFile(argv[1], star)) < 1) {
        std::cout << "Failed to open input file" << std::endl;
        return 1;
    }

    double volume, expected = std::stod(argv[2]);


    std::cout << "Processing " << argv[1] << ": ";
    volume = zonotope_volume(star, vertices, edges, dim);

    if (volume == expected)
        std::cout << "OK" << std::endl;
    else {
        std::cout << "FAIL expected " << expected << " calculated " << volume << std::endl;
        return 1;
    }
    return 0;
}
