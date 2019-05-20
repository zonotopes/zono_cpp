#include <stdio.h>
#include <stdarg.h>
#include <vector>

#include "Eigen/Dense"
#include "stplugin.h"

#include "zonotope.h"

int myprintf(const char *fmt, ...)
{
	int ret;
    char buf[1024];

    va_list args;
    va_start(args, fmt);
    ret = vsnprintf(buf, sizeof(buf), fmt, args);
    SF_display(buf);
    va_end(args);
	return ret;
}

// This is the entry point of the stata plugin (the "main" function from STATA)
STDLL stata_call(int argc, char *argv[])
{
	dbgprintf("\nSTATA_ZONOTOPE_PLUGIN_BEGIN\n");
	
	if (argc != 3)
	{
		myprintf("Invalid arguments\n");
		return 198;
	}
	

	char *mGenerators = argv[0];
	
	std::vector<Eigen::VectorXd> star;

	unsigned int cols = SF_col(mGenerators);
	unsigned int rows = SF_row(mGenerators);

	if (!((cols > 0) && (rows > 0)))
	{
		myprintf("Invalid rows/columns\n");
		return 198;
	}

	Eigen::VectorXd v(cols);
	for (unsigned int i = 1; i <= rows; i++)
	{
		for (unsigned int j = 1; j <= cols; j++ )
		{
			ST_double z;
			if (SF_mat_el(mGenerators, i, j, &z))
			{
				myprintf("Error while reading value at [%d,%d]\n", i, j);
				return 198;
			}
			v[j - 1] = z;
		}
		star.push_back(v);
	}

	std::vector<Eigen::VectorXd> vertices;
	std::vector<int> edges;
    std::vector<double> tagi;
	
	ST_double verbose;

    char *mTagi = argv[1];
	char *mStats = argv[2];
    
	SF_mat_el(mStats, 1, 1, &verbose);	
	// myprintf("verbose: %g (as int %d)\n", verbose, int(verbose));

	stats oStats = zonotope_volume(star, vertices, edges, tagi, cols, int(verbose));
			
	for (unsigned int i = 1; i <= rows; i++)
	   SF_mat_store(mTagi, i, 1, tagi[i-1]);

	SF_mat_store(mStats, 1, 1, oStats.S1);
    SF_mat_store(mStats, 1, 2, oStats.S2);
    SF_mat_store(mStats, 1, 3, oStats.S3);
	SF_mat_store(mStats, 1, 4, oStats.S4);
    SF_mat_store(mStats, 1, 5, oStats.S5);
    SF_mat_store(mStats, 1, 6, oStats.S6);
	SF_mat_store(mStats, 1, 7, oStats.S7);
    SF_mat_store(mStats, 1, 8, oStats.S8);
	SF_mat_store(mStats, 1, 9, oStats.etMIN);
    

	dbgprintf("\nSTATA_ZONOTOPE_PLUGING_END\n");
	return 0;
}

ST_plugin *_stata_ ;

STDLL pginit(ST_plugin *p)
{
	_stata_ = p ;
	return SD_PLUGINVER;
}

int main(){
	return 0;
}