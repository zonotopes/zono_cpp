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

	if (argc != 2)
	{
		myprintf("Invalid arguments\n");
		return 198;
	}

	char *mat = argv[0];
	char *scal = argv[1];

	std::vector<Eigen::VectorXd> star;

	unsigned int cols = SF_col(mat);
	unsigned int rows = SF_row(mat);

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
			if (SF_mat_el(mat, i, j, &z))
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

	double volume = zonotope_volume(star, vertices, edges, cols);

	if (SF_scal_save(scal, volume))
	{
		myprintf("Unable to save result\n");
		return 198;
	}

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