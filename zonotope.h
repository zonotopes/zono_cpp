#ifndef _ZONOTOPE_H_
#define _ZONOTOPE_H_

#include <vector>
#include "Eigen/Dense"
#include <stdarg.h>
#include "stplugin.h"

extern double zonotope_volume(std::vector<Eigen::VectorXd> &star,
    std::vector<Eigen::VectorXd> &vertices,
    std::vector<int> &edges,
    int dim);

extern unsigned int readFile(const char *filename, std::vector<Eigen::VectorXd> &star);
extern int myprintf(const char *fmt, ...);

// #define DEBUG_ZONOTOPE

#ifdef DEBUG_ZONOTOPE
#define dbgprintf(...) printf(__VA_ARGS__)
#else
#define dbgprintf(...)
#endif

#endif /* _ZONOTOPE_H_ */
