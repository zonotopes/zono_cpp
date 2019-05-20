/* ZONOTOPE LIBRARY ver 1.2

Copyright (c) 2017, Federico Ponchio.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <math.h>
#include <assert.h>

#include <fstream>
#include <iterator>
#include <algorithm>
#include <vector>
#include <set>
#include <chrono>

#include "Eigen/Dense"
#include "zonotope.h"


#ifdef __GNUC__
int __attribute__((weak)) myprintf(const char *fmt, ...)
{
	abort();
}
#endif

unsigned int readFile(const char *filename, std::vector<Eigen::VectorXd> &star) {
	std::ifstream input(filename);

	if (!input.is_open()) {
		printf("Could not open or find file: %s\n", filename);
		return 0;
	}

	std::string linestr;
	unsigned int dim = 0;

	// skipping first 3 lines
	std::getline(input, linestr);
	std::getline(input, linestr);
	std::getline(input, linestr);

	while (std::getline(input, linestr)) {
		std::istringstream buffer(linestr);
		std::vector<double> line((std::istream_iterator<double>(buffer)), std::istream_iterator<double>());

		if ((line.size() < 1) || (dim && dim != line.size())) {
			printf("Malformed input file\n");
			input.close();
			return 0;
		}

		dim = (unsigned int)(line.size());
		Eigen::VectorXd v(dim);

		for (unsigned int i = 0; i < dim; i++)
			v[i] = line.at(i);

		star.push_back(v);
	}

	input.close();

	return dim;
}

//returns -1, 1 depending on sign of val
template <typename T> int sign(T val) {
	return (T(0) <= val) - (val < T(0));
}

bool increase(int *counters, int pos, int dim, int n) {
	if(pos < 0)
		return false;

	if(counters[pos] == n - (dim - pos)) {
		return increase(counters, pos-1, dim, n);
	}

	counters[pos]++;

	for(int i = pos+1; i < dim; i++){
		counters[i] = counters[pos]+(i-pos);
	}

	return true;
}


static Eigen::VectorXd getNormal(Eigen::MatrixXd &a) { //expect a to be n*n matrix, first column is all 1, others are the n-1 generators	Vec normal(n);
	 Eigen::FullPivLU<Eigen::MatrixXd> lu(a);
	 Eigen::MatrixXd auxLU = lu.matrixLU();
	 Eigen::MatrixXd kernel = lu.kernel();
	 return kernel.block(0, 0, a.cols(), 1);
}

Eigen::VectorXd project(std::vector<int> &face, std::vector<Eigen::VectorXd> &star, int dim) {

	Eigen::VectorXd v = Eigen::VectorXd::Zero(dim);
	int n = int(star.size());
	for(int i = 0; i < n; i++) {
		v += star[i]*face[i];
	}
	return v;
}

//we should use an N-sign. hard to make though.
/*int triSign(int i, int j, int k,  vector<VectorXd> &star) {
	VectorXd d(3);
	d(0) = 1;
	d(1) = i; d(2) = i*i;
	VectorXd a = star[i] + d*1e-05;
	d(1) = j; d(2) = j*j;
	VectorXd b = star[j] + d*1e-05;
	d(1) = k; d(2) = k*k;
	VectorXd c = star[k] + d*1e-05;
	return sign((a^b)*c);
}*/

double factorial(int n) {
	double r = 1;
	for(int i = 2; i <= n; i++)
		r *= i;
	return r;
}


stats zonotope_volume(std::vector<Eigen::VectorXd> &star,
	std::vector<Eigen::VectorXd> &vertices,
	std::vector<int> &edges, std::vector<double> &tagi, int dim, bool verbose) {

	std::chrono::time_point<std::chrono::system_clock> start, end;
	start = std::chrono::system_clock::now();

	int n = int(star.size());
	int h = 0;

	std::vector<int> face(n);
	//vectors lying on the solid angle at the origin of the zonohedron
	edges.clear();

	double volume = 0;

	int *counters = new int[dim];

	for(int i = 0; i < dim-1; i++)
		counters[i] = i;
	counters[dim-1] = n;

	Eigen::MatrixXd a(dim, dim);
	Eigen::VectorXd normal;

	while (true) {

		for(int i = 0; i < dim-1; i++) {
			int col = counters[i];
			for(int row = 0; row < dim; row++)
				a(i, row) = star[col][row];
		}

		//last column as the first, we want a non invertible matrix whose kernel is the normal
		for(int row = 0; row < dim; row++)
			a(dim-1, row) = star[counters[0]][row];

		// Compute the normal of the vectors in a
		// A normal can be obtained by computing the null space of a (i.e., its kernel).
		normal = getNormal(a);
		normal.normalize();    // normalize it


		//I need to find orientation of the normal.
		for(int row = 0; row < dim; row++)
			a(dim-1, row) = normal(row);

		double s = a.determinant();

		if (s < 0)
			normal = -normal;

		int start = 0;
		for(int i = 0; i < dim; i++) {
			int end = counters[i];
			for(int k = start; k < end; k++) {
				double u = normal.dot(star[k]);
				face[k] = sign(u);
			}

			if(end < n)
				face[end] = 0;
			start = counters[i]+1;
		}

		// Find the 2^n vertices
		Eigen::VectorXd base = project(face, star, dim);

		bool compute_vertices = false;
		if (compute_vertices){
		    for(int v = 0; v < (1<<(dim-1)); v++) {
			    Eigen::VectorXd offset = Eigen::VectorXd::Zero(dim);
			    for(int i = 0; i < dim-1; i++) {           // ASK PONCHIO why the condition is "i < (dim-1)" and not "i < dim"
				    int coeff = (v & (1<<i))? 1: -1;
                    offset += star[counters[i]] * coeff;
				}

				vertices.push_back(base + offset);
				h++;
			}
		}

		double pyramid = (normal.dot(base)) * fabs(s);
		volume += pyramid;

		//increase counters
		bool keep_going = increase(counters, dim - 2, dim - 1, int(star.size()));

		if (!keep_going)
			break;
		//exit if counters[0] == n-(dim-1)
	}

	volume /= factorial(dim) / 2; //they are all N dimensional cones. (2, because we are computing half the faces, now)
	delete[] counters;

	end = std::chrono::system_clock::now();
	int64_t elapsed_msec = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();

	verbose && myprintf("------------------------------------------------------------------------\n");
    verbose && myprintf("OUTPUT VECTORS                                                         \n");
	
	// computing diagonal and total_length_squared
	Eigen::VectorXd diagonal(dim);
	for (int i = 0; i < dim; i++)
	   diagonal(i) = 0.0;
	double total_length_squared = 0;
	for (unsigned int i = 0; i < star.size(); i++) {
		 diagonal += star[i];
		 total_length_squared += star[i].squaredNorm();
	}

    verbose && myprintf("Diagonal (as a row vector):                                                              \n");
    for (int i = 0; i < dim; i++)
	   verbose && myprintf("%g ", diagonal(i));
	
	verbose && myprintf("\n");
	verbose && myprintf("Tangent between each input generator and the input space:              \n");
	
    tagi.resize(n, 0.0);
	for ( int g = 0; g < n; g++ ){
		tagi[g] = 0.0;
		double aux = 0.0;
		for ( int d=0; d < dim-1; d++ ){
            aux += star[g][d]*star[g][d];		
		}
        tagi[g] = star[g][dim-1]/sqrt(aux);
		verbose && myprintf("%g\n", tagi[g]);
	}	   
	
	verbose && myprintf("------------------------------------------------------------------------\n");
	verbose && myprintf("OUTPUT STATISTICS                              \n");
    
	stats oStats; // output stats
	oStats.S1 = volume;     
	verbose && myprintf("S1: Total volume:  %g\n", oStats.S1);

	double norm_of_the_diagonal = diagonal.norm();
	oStats.S2 = norm_of_the_diagonal;
    verbose && myprintf("S2: Diagonal norm: %g\n", oStats.S2);

    oStats.S3 = total_length_squared;
    verbose && myprintf("S3: Sum of squared norms: %g\n", total_length_squared);

    // Compute the Gini index (it depends on volume and diagonal)
    double cube = 1.0;
    for (int i = 0; i < dim; i++)
	    cube *= diagonal(i);
    double gini = volume / cube;
	oStats.S4 = gini;
    verbose && myprintf("S4: Gini index: %g\n", oStats.S4);

    double base = 0;
	for (int i = 0; i < dim - 1; i++)
	    base += diagonal(i) * diagonal(i);
    double tang_diag_input = diagonal(dim - 1) / std::sqrt(float(base));
	oStats.S5 = tang_diag_input;
    verbose && myprintf("S5: Tangent of angle btw. diagonal and the input plane: %g\n", oStats.S5);

    double cos1 = diagonal(dim - 1) / norm_of_the_diagonal;
	oStats.S6 = cos1;
    verbose && myprintf("S6: Cosine against output: %g\n", oStats.S6 );

    double sum_squared_input_diagonal = 0;
	for (int i = 0; i < dim - 1; i++)
	    sum_squared_input_diagonal += diagonal(i)*diagonal(i);

    double cos2 = diagonal(0)/std::sqrt(float( sum_squared_input_diagonal ));
	oStats.S7 = cos2;
    verbose && myprintf("S7: Cosine of proj. of diagonal on input plane with x axis: %g\n", oStats.S7);

    double vol_against_cub = volume/( norm_of_the_diagonal*norm_of_the_diagonal*norm_of_the_diagonal );
	oStats.S8 = vol_against_cub;
    verbose && myprintf("S8: Volume against the cube of the norm of the diagonal: %g\n" , oStats.S8 );

//      aux = (total_length_squared/3.0).^1.5;
//      mistery_number = binomial(size(star,1),3) * aux;
//      printf("B6: Mystery number: %g\n", mistery_number);

//      printf("B7: Tangent against input axes: %g\n", diagonal(2)/diagonal(1) );
//      printf("B8: Tangent against input axes: %g\n", xx);
//      printf("B9: Solid angle: %g\n", solidAngle(star, edges));
//      printf("B10: Normalized vectors volume: %g\n", xxx);
//      printf("B11: Volume against diagonal cubed of boundary vectors: %g \n", xxxx);
    
	verbose && myprintf("------------------------------------------------------------------------\n");
    verbose && myprintf("Elapsed time (MIN):\n");
    verbose && myprintf("%f\n", elapsed_msec / 60000.0f);
    verbose && myprintf("------------------------------------------------------------------------\n");

    oStats.etMIN = elapsed_msec / 60000.0f;
	return oStats;

}
