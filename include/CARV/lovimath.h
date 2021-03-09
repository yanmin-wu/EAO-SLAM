#ifndef __LOVIMATH_H
#define __LOVIMATH_H

#include <cmath>
#include <limits>

//#include <wnlib/wnconj.h>

using namespace std;

namespace dlovi{
  //#ifndef EPS
  //	#define EPS (10e-6)
  //#endif

  //#ifndef PI
  //	#define PI (3.141592653589793238462643)
  //#endif

	// General mathematical constants
	const double pi = 3.141592653589793238462643;
	const double ln2 = 0.69314718055994530942;
	const double e = 2.718281828459045235360287;

	// Constants for golden section searches, etc
	const double golden_ratio = 1.6180339887498948482;
	const double golden_ratio_conjugate = 0.618033988749895;
	const double golden_section = 0.381966011250105;

	// Machine precision related constants
	const double eps_d = pow(2.0, -52.0);
	const double eps_f = pow(2.0, -23.0);
	const double sqrt_eps_d = 1.49011611938477e-08;
	const double sqrt_eps_f = 3.45266983001244e-04;
	const double realmin_d = ldexp(1.0, -1022);
	const double realmin_f = (float)ldexp(1.0, -126);

	// Constants related to Inf and NaN
	const double NaN = numeric_limits<double>::quiet_NaN();
	const double Inf = numeric_limits<double>::infinity();
	const double inf = numeric_limits<double>::infinity();

	// (Most) Functions Declarations
	int floatEquals(float, float);
	int doubleEquals(double, double);

	double eps(double x);
	float eps(float x);

	int round(double x);
	int round(float x);

	bool isNaN(double x);

	// Special functions
	double sinc(double x);

	// Templated functions:

	template <class T> 
	T sign(const T x){ // computes the signum
		if(x > T(0))
			return T(1);
		else if(x < T(0))
			return T(-1);
		else
			return T(0);
	}

	template <class T>
	T replaceSign(const T a, const T b){ // returns a value with the same magnitude as a and the same sign as b.  (if sign(b) == 0, sign of a unchanged)
		T nSignb = sign(b);
		T nSigna = sign(a); 
		return a * nSigna * ( (nSignb == T(0)) ? nSigna : nSignb );
	}
}
#endif

