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

    if (argc != 2) {
        std::cerr << "Invalid arguments" << std::endl;
        return 1;
    }

	std::ifstream input(argv[1]);

	if (!input.is_open()) {
		std::cerr << "Could not open or find file: " << argv[1] << std::endl;
		return 0;
	}

    std::string datadir = std::string(argv[1]);
    std::replace(datadir.begin(), datadir.end(), '\\', '/');
    datadir.erase(std::find(datadir.rbegin(), datadir.rend(), '/').base(), datadir.end());

    std::cout << "Datadir is: " << datadir << std::endl << std::endl;

	unsigned int dim;
    std::vector<Eigen::VectorXd> star, vertices;
	std::vector<int> edges;
    std::vector<double> tagi;
    
    std::string linestr;
    std::string filename;
    double expected, volume;
    int failed = 0;

    while (std::getline(input, linestr)) {
        std::istringstream iss(linestr);
        iss >> filename;

        if (filename.at(0) == '#')
            continue;

        filename = datadir + filename;

        iss >> expected;
        std::cout << "Processing " << filename << ": ";

        star.clear();
        vertices.clear();
        edges.clear();
        tagi.clear();

    	if ((dim = readFile(filename.c_str(), star)) < 1) {
            std::cout << "FAIL Failed to open input file" << std::endl;
            return 1;
        }

        stats oStats = zonotope_volume(star, vertices, edges, tagi, dim);

        volume = oStats.S1;


        if (volume == expected)
            std::cout << "OK" << std::endl;
        else {
            std::cout << "FAIL expected " << expected << " calculated " << volume << std::endl;
            failed++;
            return 1; // comment out if you want to keep going
        }
    }

    return (failed == 0) ? 0 : 1;
}
