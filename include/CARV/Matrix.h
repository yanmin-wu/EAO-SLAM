#ifndef __MATRIX_H
#define __MATRIX_H

#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include "CARV/lovimath.h"

using namespace std;

#include "CARV/lapack_declarations.h"

namespace dlovi{

	enum ConvolutionType{
		CONV_FULL=0,
		CONV_SAME=10,
		CONV_VALID=20};

	enum PNormType{
		PNORM_ONE=1,
		PNORM_TWO=2,
		PNORM_FROBENIUS=500,
		PNORM_INF=10000,
		PNORM_NEG_INF=-10000};

	enum FSpecialType{
		FSPECIAL_GAUSSIAN=0,
		FSPECIAL_SOBEL=10,
		FSPECIAL_PREWITT=20,
		FSPECIAL_LAPLACIAN=30,
		FSPECIAL_LOG=40,
		FSPECIAL_AVERAGE=50,
		FSPECIAL_UNSHARP=60};

	class Matrix;
	class Matrix{
	public:
		// Constructors
		Matrix();
		Matrix(const int rows, const int cols, const double fillval = 0.0);
		Matrix(const Matrix & ref);
		Matrix(const vector<double> & ref);
		Matrix(const double * ref, const int length);
		Matrix(const double ref);
		Matrix(string strMatrix);
		Matrix(istream & stream);
		// TODO: add a constructor that takes a variable # of double arguments and returns a row-vector of those doubles.  Perhaps another one w/ a bool flag preceeding it.
		          // does above idea  conflicts with constructor #2?

		// Getters
		int numRows() const;
		int numCols() const;
		int numElements() const;
		double at(const int index) const;
		double at(const int row, const int col) const;
		Matrix at(const vector<int> & rows, const vector<int> & cols) const;
		Matrix at(const int startrow, const int steprow, const int endrow, const int startcol, const int stepcol,
			const int endcol) const;
		Matrix at(const int startrow, const int endrow, const int startcol, const int endcol) const;

		// Setters
		void set(const int index, const double val);
		void set(const int row, const int col, const double val);
		void set(const vector<int> & rows, const vector<int> & cols, const double val);
		void set(const vector<int> & rows, const vector<int> & cols, const Matrix & val);
		void set(const int startrow, const int steprow, const int endrow, const int startcol, const int stepcol,
			const int endcol, const double val);
		void set(const int startrow, const int steprow, const int endrow, const int startcol, const int stepcol,
			const int endcol, const Matrix & val);
		void set(const int startrow, const int endrow, const int startcol, const int endcol, const double val);
		void set(const int startrow, const int endrow, const int startcol, const int endcol, const Matrix & val);

		// Operators
		Matrix & operator=(const Matrix & rhs);

		Matrix operator+() const;
		Matrix operator-() const;
		Matrix operator+(const Matrix & rhs) const;
		Matrix operator+(const double rhs) const;
		Matrix operator-(const Matrix & rhs) const;
		Matrix operator-(const double rhs) const;
		Matrix operator*(const Matrix & rhs) const;
		Matrix operator*(const double rhs) const;
		Matrix operator/(const double rhs) const;
		Matrix operator^(const Matrix & rhs) const;
		Matrix operator^(const double rhs) const;

		Matrix & operator+=(const Matrix & rhs);
		Matrix & operator+=(const double rhs);
		Matrix & operator-=(const Matrix & rhs);
		Matrix & operator-=(const double rhs);
		Matrix & operator*=(const Matrix & rhs);
		Matrix & operator*=(const double rhs);
		Matrix & operator/=(const double rhs);
		Matrix & operator^=(const double rhs);

		Matrix operator==(const Matrix & rhs) const;
		Matrix operator==(const double rhs) const;
		Matrix operator!=(const Matrix & rhs) const;
		Matrix operator!=(const double rhs) const;
		Matrix operator>=(const Matrix & rhs) const;
		Matrix operator>=(const double rhs) const;
		Matrix operator<=(const Matrix & rhs) const;
		Matrix operator<=(const double rhs) const;
		Matrix operator>(const Matrix & rhs) const;
		Matrix operator>(const double rhs) const;
		Matrix operator<(const Matrix & rhs) const;
		Matrix operator<(const double rhs) const;

		operator double*();

		const double & operator()(const int index) const;
		double & operator()(const int index);
		const double & operator()(const int row, const int col) const;
		double & operator()(const int row, const int col);
		Matrix operator()(const vector<int> & rows, const vector<int> & cols) const;
		Matrix operator()(const int startrow, const int steprow, const int endrow, const int startcol, const int stepcol,
			const int endcol) const;
		Matrix operator()(const int startrow, const int endrow, const int startcol, const int endcol) const;

		// Public Methods
		void resize(const int rows, const int cols);
		void reserve(const int n);
		void reserve(const int rows, const int cols);
		void reshape(const int rows, const int cols);
		void clear();
		void fill(const double fillval);
		double det() const;
		Matrix codedDet() const;
		int rank() const;
		int rank(const double tol) const;
		Matrix inv() const;
		Matrix mldivide(const Matrix & rhs) const;
		Matrix mrdivide(const Matrix & rhs) const;
		Matrix times(const Matrix & rhs) const;
		Matrix ldivide(const Matrix & rhs) const;
		Matrix rdivide(const Matrix & rhs) const;
		Matrix solveHomogeneous() const;
		Matrix null() const;
		Matrix transpose() const;
		Matrix sum() const;
		Matrix sum(const int dim) const;
		double sumAll() const;
		Matrix prod() const;
		Matrix prod(const int dim) const;
		double prodAll() const;
		Matrix mean() const;
		Matrix mean(const int dim) const;
		double meanAll() const;
		Matrix var() const;
		Matrix var(const int dim) const;
		double varAll() const;
		Matrix max() const;
		Matrix max(const Matrix & rhs) const;
		Matrix max(const double rhs) const;
		double maxAll() const;
		Matrix min() const;
		Matrix min(const Matrix & rhs) const;
		Matrix min(const double rhs) const;
		double minAll() const;
		Matrix diag() const;
		double trace() const;
		Matrix abs() const;
		double dot(const Matrix & rhs) const;
		Matrix cross(const Matrix & rhs) const;
		double angleBetween(const Matrix & rhs) const;
		double norm(const PNormType normType= PNORM_TWO) const;
		double norm(const double p) const;
		Matrix tril() const;
		Matrix triu() const;
		void LU(Matrix & L, Matrix & U) const;
		void LU(Matrix & L, Matrix & U, Matrix & P) const;
		void LU(Matrix & L, Matrix & U, vector<int> & piv) const;
		void QR(Matrix & Q, Matrix & R) const;
		void QR(Matrix & Q, Matrix & R, Matrix & P) const;
		Matrix SVD() const;
		void SVD(Matrix & U, Matrix & S, Matrix & V) const;
		bool isColVector() const;
		bool isRowVector() const;
		bool isVector() const;
		bool isEmpty() const;
		string toString() const;
		string toInfoString() const;
		Matrix & hcat(const Matrix & rhs);
		Matrix & vcat(const Matrix & rhs);
		Matrix pflat() const;
		Matrix conv(const Matrix & rhs) const;
		Matrix conv2(const Matrix & rhs, const ConvolutionType type=CONV_FULL) const;
		Matrix filter2(const Matrix & rhs, const ConvolutionType type=CONV_SAME) const;

		// Public Static Methods
		static Matrix eye(const int size);
		static Matrix eye(const int rows, const int cols);
		static Matrix zeros(const int rows, const int cols);
		static Matrix ones(const int rows, const int cols);
		static Matrix diag(const int size, const double val);
		static Matrix diag(const vector<double> & vals);
		static Matrix exp(const Matrix & rhs);
		static Matrix fspecial(const FSpecialType type, const Matrix & size = Matrix("[3, 3]"), const double param1=0.5);
		static Matrix rot2D(const double theta);
		static Matrix trans2D(const double x, const double y);
		static Matrix rotx3D(const double theta);
		static Matrix roty3D(const double theta);
		static Matrix rotz3D(const double theta);
		static Matrix rot3D(const Matrix & t); // Axis-angle vector t
		static Matrix rot3D(const double theta, const Matrix & t); // Rodrigues formula (angle and unit axis-vector)
		static Matrix applyRot3D(const Matrix & t, const Matrix & x); // Axis-angle vector t
		static Matrix applyRot3D(const double theta, const Matrix & t, const Matrix & x); // Rodrigues formula (angle and unit axis-vector)
		static Matrix axisAngleFromRot3D(const Matrix & R); // Computes axis-angle vector t from R
		static void axisAngleFromRot3D(Matrix & v, double & theta, const Matrix & R); // Computes axis vector v and angle theta from R
		static Matrix scale3D(const double sx, const double sy, const double sz);
		static Matrix trans3D(const double x, const double y, const double z);
		static Matrix skewSym(const Matrix & t);

	private:
		// :::: private vect<T> class implementation ::::
		#include "CARV/Matrix_vect_private.cc"

		// Private Methods
		int rank(const Matrix & s) const;
		int rank(const Matrix & s, const double tol) const;

		// Getters
		//const vect<double> & GetData() const;

		// Setters
		//void SetData(const vect<double> & ref);

		// Members
		vect<double> m_Data;
		int m_nCols;
		int m_nRows;
	};

	// Left hand binary operators
	Matrix operator+(const double lhs, const Matrix & rhs);
	Matrix operator-(const double lhs, const Matrix & rhs);
	Matrix operator*(const double lhs, const Matrix & rhs);
	Matrix operator/(const double lhs, const Matrix & rhs);
	Matrix operator^(const double lhs, const Matrix & rhs);

	Matrix operator==(const double lhs, const Matrix & rhs);
	Matrix operator!=(const double lhs, const Matrix & rhs);
	Matrix operator>=(const double lhs, const Matrix & rhs);
	Matrix operator<=(const double lhs, const Matrix & rhs);
	Matrix operator>(const double lhs, const Matrix & rhs);
	Matrix operator<(const double lhs, const Matrix & rhs);
}

#endif
