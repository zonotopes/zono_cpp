#ifndef _ZONOTOPE_H_
#define _ZONOTOPE_H_

#include <vector>
#include "Eigen/Dense"
#include <stdarg.h>
#include "stplugin.h"

// data structure for returning output statistics
struct stats{
	double S1,S2,S3,S4,S5,S6,S7,S8,etMIN; // the last is the elabsed time (in minutes)
};

extern stats zonotope_volume(std::vector<Eigen::VectorXd> &star,
    std::vector<Eigen::VectorXd> &vertices,
    std::vector<int> &edges,
    std::vector<double> &tagi,
    int dim,
    bool verbose = false);
    
extern unsigned int readFile(const char *filename, std::vector<Eigen::VectorXd> &star);
extern int myprintf(const char *fmt, ...);


// uncomment next line if you want to print debug information during the call of the stata plugin
// #define DEBUG_ZONOTOPE

#ifdef DEBUG_ZONOTOPE
#define dbgprintf(...) myprintf(__VA_ARGS__)
#else
#define dbgprintf(...)
#endif

#endif /* _ZONOTOPE_H_ */
