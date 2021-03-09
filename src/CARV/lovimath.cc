#ifndef __LOVIMATH_CPP
#define __LOVIMATH_CPP

#include "CARV/lovimath.h"

namespace dlovi{

	int floatEquals(float x, float y){
		return fabs(x - y) < eps_f;
	}

	int doubleEquals(double x, double y){
		return fabs(x - y) < eps_d;
	}

	double eps(double x){
		if (fabs(x) <= realmin_d)
			return pow(2.0, -1074.0);
		else{
			//2^E <= abs(X) < 2^(E+1)  -->  E <= log_2(abs(X)) < E+1....
			//true iff E = floor(log_2(abs(X))).  Floor can be done by casting 	//to int.
			//so......
			//int E = (int)log_2(fabs(x));

			return pow(2.0, (double)((int)(log(fabs(x)/ln2)) - 52));
		}
	}

	float eps(float x){
		if (fabs(x) <= realmin_f)
			return pow(2.0, -149.0);
		else{
			//2^E <= abs(X) < 2^(E+1)  -->  E <= log_2(abs(X)) < E+1....
			//true iff E = floor(log_2(abs(X))).  Floor can be done by casting 	//to int.
			//so......
			//int E = (int)log_2(fabs(x));

			return (float)pow(2.0, (double)((int)(log(fabs(x)/ln2)) - 23));
		}
	}

	int round(double x){
		if(x >= 0.0)
			return (int)(x + 0.5);
		return (int)(x - 0.5);
	}

	int round(float x){
		if(x >= 0.0f)
			return (int)(x + 0.5f);
		return (int)(x - 0.5f);
	}

	bool isNaN(double x){
		return x != x;
	}

	double sinc(double x){
		if(x == 0.0)
			return 1;
		return sin(x) / x;
	}
}

#endif
