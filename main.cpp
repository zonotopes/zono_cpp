#include <stdio.h>
#include <vector>

#include "Eigen/Dense"
#include "zonotope.h"

int myprintf(const char *fmt, ...)
{
	int ret;
	va_list arglist;
	va_start(arglist, fmt);
	ret = vprintf(fmt, arglist);
	va_end(arglist);
	return ret;
}

int main(int argc, char *argv[]) {

	unsigned int dim;
	std::vector<Eigen::VectorXd> star;

	char *data_file_name = argv[1];

	dbgprintf("\nZONOTOPE_MAIN_BEGIN\n\n");

    if (data_file_name == 0){

		printf("\nNo file name specified. Using a demo  dataset in 3D\n\n");

		dim = 3;
		Eigen::VectorXd v(dim);

	    v[0] = 9.7974838e-01;   v[1] = 6.2406009e-01;  v[2] = 6.0986665e-01;
	    star.push_back(v);
	    v[0] = 4.3886997e-01;   v[1] = 6.7913554e-01;  v[2] = 6.1766639e-01;
	    star.push_back(v);
	    v[0] = 1.1111922e-01;   v[1] = 3.9551522e-01;  v[2] = 8.5944231e-01;
	    star.push_back(v);
	    v[0] = 2.5806470e-01;   v[1] = 3.6743665e-01;  v[2] = 8.0548942e-01;
	    star.push_back(v);
	    v[0] = 4.0871985e-01;   v[1] = 9.8798200e-01;  v[2] = 5.7672152e-01;
	    star.push_back(v);
	    v[0] = 5.9489607e-01;   v[1] = 3.7738866e-02;  v[2] = 1.8292247e-01;
	    star.push_back(v);
	    v[0] = 2.6221175e-01;   v[1] = 8.8516801e-01;  v[2] = 2.3993201e-01;
	    star.push_back(v);
	    v[0] = 6.0284309e-01;   v[1] = 9.1328683e-01;  v[2] = 8.8651193e-01;
	    star.push_back(v);
	    v[0] = 7.1121578e-01;   v[1] = 7.9618387e-01;  v[2] = 2.8674152e-02;
	    star.push_back(v);
	    v[0] = 2.2174673e-01;   v[1] = 9.8712279e-02;  v[2] = 4.8990139e-01;
	    star.push_back(v);
    }else{
		if ((dim = readFile(data_file_name, star)) < 1)
		return 1;
	}

	std::vector<Eigen::VectorXd> vertices;
	std::vector<int> edges;

	printf("-----------------------------------------------------------------------\n");
	printf("ZONOTOPE LIBRARY VER 1.2                       \n");
	printf("-----------------------------------------------------------------------\n");
	printf("INPUT: SET OF GENERATORS             \n");
	printf("N. of dimensions (including the output): %d\n", dim );
	printf("N. of generators: %d\n", star.size() );
	myprintf("-----------------------------------------------------------------------\n");
	printf("... the computation of the volume has started (it can take a while) ... \n");

	double volume = zonotope_volume(star, vertices, edges, dim);

	// printf("Volume of the zonotope: %g\n", volume);

	dbgprintf("\nZONOTOPE_MAIN_END\n"); // KUKOC_ADDED!!!

	return 0;
}
