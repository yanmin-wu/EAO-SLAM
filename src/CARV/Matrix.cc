#ifndef __MATRIX_CPP
#define __MATRIX_CPP

#include "CARV/Matrix.h"

namespace dlovi{

	// Constructors

	Matrix::Matrix(){
		m_nCols = 0;
		m_nRows = 0;
	}

	Matrix::Matrix(const int rows, const int cols, const double fillval){
		m_Data = vect<double>(rows*cols, fillval);
		m_nRows = rows;
		m_nCols = cols;
	}

	Matrix::Matrix(const Matrix & ref){
		//SetData(ref.GetData());
		m_Data = ref.m_Data;
		m_nRows = ref.numRows();
		m_nCols = ref.numCols();
	}

	Matrix::Matrix(const vector<double> & ref){
		m_Data = vect<double>((signed int)ref.size());
		m_nRows = (signed int)ref.size();
		m_nCols = 1;
		for(int i = 0; i < m_nRows; i++)
			m_Data[i] = ref[i];
	}

	Matrix::Matrix(const double * ref, const int length){
		m_Data = vect<double>(length);
		m_nRows = length;
		m_nCols = 1;
		for(int i = 0; i < m_nRows; i++)
			m_Data[i] = ref[i];
	}

	Matrix::Matrix(const double ref){
		m_Data = vect<double>(1);
		m_nRows = 1;
		m_nCols = 1;
		m_Data[0] = ref;
	}

	Matrix::Matrix(string strMatrix){
		istringstream iss(strMatrix);
		char junk;
		double tmp;

		// Swift check for empty matrix
		iss >> junk;
		iss >> junk;
		if(junk == ']'){
			m_nCols = 0;
			m_nRows = 0;	
			return;
		}
		iss.str(strMatrix);

		// read in first line
		iss >> junk;
		while(junk != ';' && junk != ']'){
			iss >> tmp;
			iss >> junk;

			m_Data.push_back(tmp);
		}

		m_nCols = (int)m_Data.size();
		m_nRows = 1;

		// if done, return
		if(junk == ']')
			return;

		// else read in the rest
		while (junk != ']'){
			junk = (char)0;
			while(junk != ';' && junk != ']'){
				iss >> tmp;
				iss >> junk;

				m_Data.push_back(tmp);
			}
			m_nRows++;
		}

		// Convert to column-major order in internal data representation.
		int szTmp = m_nCols;
		m_nCols = m_nRows;
		m_nRows = szTmp;
		*this = transpose();
	}

	Matrix::Matrix(istream & stream){
		// TODO: check for valid input (eg: eof)
		// TODO: make more efficient (the stringstream is probably unnecessary)
		ostringstream o;
		char cTmp = '\0';
		while(cTmp != '['){
			// skip to the first [
			stream >> cTmp;
			if(cTmp == '[')
				o << cTmp;
		}
		while(cTmp != ']'){
			// read until the first ]
			stream >> cTmp;
			// TODO: add a whitespace-skipping loop here (for multiple consecutive redundant whitespaces).
			if(cTmp != '\n' && cTmp != '\r')
				o << cTmp;
		}

		istringstream iss(o.str());
		char junk;
		double tmp;

		// Swift check for empty matrix
		iss >> junk;
		iss >> junk;
		if(junk == ']'){
			m_nCols = 0;
			m_nRows = 0;
			return;
		}
		iss.str(o.str());

		// read in first line
		iss >> junk;
		while(junk != ';' && junk != ']'){
			iss >> tmp;
			iss >> junk;

			m_Data.push_back(tmp);
		}

		m_nCols = (int)m_Data.size();
		m_nRows = 1;

		// if done, return
		if(junk == ']')
			return;

		// else read in the rest
		while (junk != ']'){
			junk = (char)0;
			while(junk != ';' && junk != ']'){
				iss >> tmp;
				iss >> junk;

				m_Data.push_back(tmp);
			}
			m_nRows++;
		}

		// Convert to column-major order in internal data representation.
		int szTmp = m_nCols;
		m_nCols = m_nRows;
		m_nRows = szTmp;
		*this = transpose();
	}

	// Getters

	int Matrix::numRows() const{
		return m_nRows;
	}

	int Matrix::numCols() const{
		return m_nCols;
	}

	int Matrix::numElements() const{
		return m_nRows * m_nCols;
	}

	double Matrix::at(const int index) const{
		return m_Data[index];
	}

	double Matrix::at(const int row, const int col) const{
		return m_Data[col * m_nRows + row];
	}

	Matrix Matrix::at(const vector<int> & rows, const vector<int> & cols) const{
		int i, j;
		Matrix retVal((int)rows.size(), (int)cols.size());

		for(i = 0; i < (int)rows.size(); i++){
			for(j = 0; j < (int)cols.size(); j++){
				retVal.set(i, j, m_Data[cols[j] * m_nRows + rows[i]]);
			}
		}

		return retVal;
	}

	Matrix Matrix::at(const int startrow, const int steprow, const int endrow, const int startcol, const int stepcol,
		const int endcol) const{
		// TODO: implement negative steps for all the at() and operator()'s.

		int i, j;
		Matrix retVal(((endrow - startrow) / steprow) + 1, ((endcol - startcol) / stepcol) + 1);

		if(steprow > 0 && stepcol > 0){
			for(i = startrow; i <= endrow; i += steprow){
				for(j = startcol; j <= endcol; j += stepcol){
					retVal.set((i-startrow) / steprow, (j-startcol) / stepcol, m_Data[j * m_nRows + i]);
				}
			}
		}
		else if(steprow > 0 && stepcol < 0){
			for(i = startrow; i <= endrow; i += steprow){
				for(j = startcol; j >= endcol; j += stepcol){
					retVal.set((i-startrow) / steprow, (j-startcol) / stepcol, m_Data[j * m_nRows + i]);
				}
			}
		}
		else if(steprow < 0 && stepcol > 0){
			for(i = startrow; i >= endrow; i += steprow){
				for(j = startcol; j <= endcol; j += stepcol){
					retVal.set((i-startrow) / steprow, (j-startcol) / stepcol, m_Data[j * m_nRows + i]);
				}
			}
		}
		else{
			for(i = startrow; i >= endrow; i += steprow){
				for(j = startcol; j >= endcol; j += stepcol){
					retVal.set((i-startrow) / steprow, (j-startcol) / stepcol, m_Data[j * m_nRows + i]);
				}
			}
		}

		return retVal;
	}

	Matrix Matrix::at(const int startrow, const int endrow, const int startcol, const int endcol) const{
		int i, j;
		Matrix retVal(endrow - startrow + 1, endcol - startcol + 1);

		for(i = startrow; i <= endrow; i++){
			for(j = startcol; j <= endcol; j++){
				retVal.set(i-startrow, j-startcol, m_Data[j * m_nRows + i]);
			}
		}

		return retVal;
	}

	/*const vect<double> & Matrix::GetData() const{
		return m_Data;
	}*/

	// Setters

	void Matrix::set(const int index, const double val){
		m_Data[index] = val;
	}

	void Matrix::set(const int row, const int col, const double val){
		m_Data[col * m_nRows + row] = val;
	}

	void Matrix::set(const vector<int> & rows, const vector<int> & cols, const double val){
		int i, j;
		for(i = 0; i < (int)rows.size(); i++){
			for(j = 0; j < (int)cols.size(); j++){
				m_Data[cols[j] * m_nRows + rows[i]] = val;
			}
		}
	}

	void Matrix::set(const vector<int> & rows, const vector<int> & cols, const Matrix & val){
		int i, j;
		for(i = 0; i < (int)rows.size(); i++){
			for(j = 0; j < (int)cols.size(); j++){
				m_Data[cols[j] * m_nRows + rows[i]] = val.at(i, j);
			}
		}
	}

	void Matrix::set(const int startrow, const int steprow, const int endrow, const int startcol, const int stepcol, const int endcol,
		const double val){	

		int i, j;
		for(i = startrow; i <= endrow; i += steprow){
			for(j = startcol; j <= endcol; j += stepcol){
				m_Data[j * m_nRows + i] = val;
			}
		}
	}

	void Matrix::set(const int startrow, const int steprow, const int endrow, const int startcol, const int stepcol, const int endcol,
		const Matrix & val){	

		int i, j;
		for(i = startrow; i <= endrow; i += steprow){
			for(j = startcol; j <= endcol; j += stepcol){
				m_Data[j * m_nRows + i] = val.at((i-startrow) / steprow, (j-startcol) / stepcol);
			}
		}
	}

	void Matrix::set(const int startrow, const int endrow, const int startcol, const int endcol, const double val){
		int i, j;
		for(i = startrow; i <= endrow; i++){
			for(j = startcol; j <= endcol; j++){
				m_Data[j * m_nRows + i] = val;
			}
		}
	}

	void Matrix::set(const int startrow, const int endrow, const int startcol, const int endcol, const Matrix & val){
		int i, j;
		for(i = startrow; i <= endrow; i++){
			for(j = startcol; j <= endcol; j++){
				m_Data[j * m_nRows + i] = val.at(i-startrow, j-startcol);
			}
		}
	}

	/*void Matrix::SetData(const vect<double> & ref){
		m_Data = ref;
	}*/

	// Operators

	Matrix & Matrix::operator=(const Matrix & rhs){
		if(this != & rhs){
			//SetData(rhs.GetData());
			m_Data = rhs.m_Data;
			m_nRows = rhs.numRows();
			m_nCols = rhs.numCols();
		}
		return *this;
	}

	Matrix Matrix::operator+() const{
		return *this;
	}

	Matrix Matrix::operator-() const{
		int i, j;
		Matrix retVal(m_nRows, m_nCols);

		for(i = 0; i < m_nRows; i++){
			for(j = 0; j < m_nCols; j++){
				retVal.set(i, j, -at(i, j));
			}
		}

		return retVal;
	}

	Matrix Matrix::operator+(const Matrix & rhs) const{
		int i, j;
		Matrix retVal(m_nRows, m_nCols);

		for(i = 0; i < m_nRows; i++){
			for(j = 0; j < m_nCols; j++){
				retVal.set(i, j, at(i, j) + rhs.at(i, j));
			}
		}

		return retVal;
	}

	Matrix Matrix::operator+(const double rhs) const{
		int i, j;
		Matrix retVal(m_nRows, m_nCols);

		for(i = 0; i < m_nRows; i++){
			for(j = 0; j < m_nCols; j++){
				retVal.set(i, j, at(i, j) + rhs);
			}
		}

		return retVal;
	}

	Matrix Matrix::operator-(const Matrix & rhs) const{
		int i, j;
		Matrix retVal(m_nRows, m_nCols);

		for(i = 0; i < m_nRows; i++){
			for(j = 0; j < m_nCols; j++){
				retVal.set(i, j, at(i, j) - rhs.at(i, j));
			}
		}

		return retVal;
	}

	Matrix Matrix::operator-(const double rhs) const{
		int i, j;
		Matrix retVal(m_nRows, m_nCols);

		for(i = 0; i < m_nRows; i++){
			for(j = 0; j < m_nCols; j++){
				retVal.set(i, j, at(i, j) - rhs);
			}
		}

		return retVal;
	}

	Matrix Matrix::operator*(const Matrix & rhs) const{
		int i, j, k;
		double tmp;
		Matrix retVal(m_nRows, rhs.m_nCols);

		for(i = 0; i < m_nRows; i++){
			for(j = 0; j < rhs.m_nCols; j++){
				tmp = 0;
				for (k = 0; k < m_nCols; k++){
					tmp += at(i, k) * rhs.at(k, j);
				}
				retVal.set(i, j, tmp);
			}
		}

		return retVal;
	}

	Matrix Matrix::operator*(const double rhs) const{
		int i, j;
		Matrix retVal(m_nRows, m_nCols);

		for(i = 0; i < m_nRows; i++){
			for(j = 0; j < m_nCols; j++){
				retVal.set(i, j, at(i, j) * rhs);
			}
		}

		return retVal;
	}

	Matrix Matrix::operator/(const double rhs) const{
		int i, j;

		Matrix retVal(m_nRows, m_nCols);

		for(i = 0; i < m_nRows; i++){
			for(j = 0; j < m_nCols; j++){
				retVal.set(i, j, at(i, j) / rhs);
			}
		}

		return retVal;
	}

	Matrix Matrix::operator^(const Matrix & rhs) const{
		int i, j;

		Matrix retVal(m_nRows, m_nCols);

		for(i = 0; i < m_nRows; i++){
			for(j = 0; j < m_nCols; j++){
				retVal.set(i, j, pow(at(i, j), rhs(i, j)));
			}
		}

		return retVal;
	}

	Matrix Matrix::operator^(const double rhs) const{
		int i, j;

		Matrix retVal(m_nRows, m_nCols);

		for(i = 0; i < m_nRows; i++){
			for(j = 0; j < m_nCols; j++){
				retVal.set(i, j, pow(at(i, j), rhs));
			}
		}

		return retVal;
	}

	Matrix & Matrix::operator+=(const Matrix & rhs){
		return *this = *this + rhs;
	}

	Matrix & Matrix::operator+=(const double rhs){
		return *this = *this + rhs;
	}

	Matrix & Matrix::operator-=(const Matrix & rhs){
		return *this = *this - rhs;
	}

	Matrix & Matrix::operator-=(const double rhs){
		return *this = *this - rhs;
	}

	Matrix & Matrix::operator*=(const Matrix & rhs){
		return *this = *this * rhs;
	}

	Matrix & Matrix::operator*=(const double rhs){
		return *this = *this * rhs;
	}

	Matrix & Matrix::operator/=(const double rhs){
		return *this = *this / rhs;
	}

	Matrix & Matrix::operator^=(const double rhs){
		return *this = *this ^ rhs;
	}

	Matrix Matrix::operator==(const Matrix & rhs) const{
		int i, j;
		Matrix retVal(m_nRows, m_nCols);

		for(i = 0; i < m_nRows; i++){
			for(j = 0; j < m_nCols; j++){
				retVal.set(i, j, doubleEquals(at(i, j), rhs.at(i, j)));
			}
		}
		return retVal;
	}

	Matrix Matrix::operator==(const double rhs) const{
		int i, j;
		Matrix retVal(m_nRows, m_nCols);

		for(i = 0; i < m_nRows; i++){
			for(j = 0; j < m_nCols; j++){
				retVal.set(i, j, doubleEquals(at(i, j), rhs));
			}
		}
		return retVal;
	}

	Matrix Matrix::operator!=(const Matrix & rhs) const{
		int i, j;
		Matrix retVal(m_nRows, m_nCols);

		for(i = 0; i < m_nRows; i++){
			for(j = 0; j < m_nCols; j++){
				retVal.set(i, j, !doubleEquals(at(i, j), rhs.at(i, j)));
			}
		}
		return retVal;
	}

	Matrix Matrix::operator!=(const double rhs) const{
		int i, j;
		Matrix retVal(m_nRows, m_nCols);

		for(i = 0; i < m_nRows; i++){
			for(j = 0; j < m_nCols; j++){
				retVal.set(i, j, !doubleEquals(at(i, j), rhs));
			}
		}
		return retVal;
	}

	Matrix Matrix::operator>=(const Matrix & rhs) const{
		int i, j;
		Matrix retVal(m_nRows, m_nCols);

		for(i = 0; i < m_nRows; i++){
			for(j = 0; j < m_nCols; j++){
				retVal.set(i, j, at(i, j) >= rhs.at(i, j));
			}
		}
		return retVal;
	}

	Matrix Matrix::operator>=(const double rhs) const{
		int i, j;
		Matrix retVal(m_nRows, m_nCols);

		for(i = 0; i < m_nRows; i++){
			for(j = 0; j < m_nCols; j++){
				retVal.set(i, j, at(i, j) >= rhs);
			}
		}
		return retVal;
	}

	Matrix Matrix::operator<=(const Matrix & rhs) const{
		int i, j;
		Matrix retVal(m_nRows, m_nCols);

		for(i = 0; i < m_nRows; i++){
			for(j = 0; j < m_nCols; j++){
				retVal.set(i, j, at(i, j) <= rhs.at(i, j));
			}
		}
		return retVal;
	}

	Matrix Matrix::operator<=(const double rhs) const{
		int i, j;
		Matrix retVal(m_nRows, m_nCols);

		for(i = 0; i < m_nRows; i++){
			for(j = 0; j < m_nCols; j++){
				retVal.set(i, j, at(i, j) <= rhs);
			}
		}
		return retVal;
	}

	Matrix Matrix::operator>(const Matrix & rhs) const{
		int i, j;
		Matrix retVal(m_nRows, m_nCols);

		for(i = 0; i < m_nRows; i++){
			for(j = 0; j < m_nCols; j++){
				retVal.set(i, j, at(i, j) > rhs.at(i, j));
			}
		}
		return retVal;
	}

	Matrix Matrix::operator>(const double rhs) const{
		int i, j;
		Matrix retVal(m_nRows, m_nCols);

		for(i = 0; i < m_nRows; i++){
			for(j = 0; j < m_nCols; j++){
				retVal.set(i, j, at(i, j) > rhs);
			}
		}
		return retVal;
	}

	Matrix Matrix::operator<(const Matrix & rhs) const{
		int i, j;
		Matrix retVal(m_nRows, m_nCols);

		for(i = 0; i < m_nRows; i++){
			for(j = 0; j < m_nCols; j++){
				retVal.set(i, j, at(i, j) < rhs.at(i, j));
			}
		}
		return retVal;
	}

	Matrix Matrix::operator<(const double rhs) const{
		int i, j;
		Matrix retVal(m_nRows, m_nCols);

		for(i = 0; i < m_nRows; i++){
			for(j = 0; j < m_nCols; j++){
				retVal.set(i, j, at(i, j) < rhs);
			}
		}
		return retVal;
	}

	Matrix::operator double*(){
		return m_Data.GetData();
	}

	const double & Matrix::operator()(const int index) const{
		return m_Data[index];
	}

	double & Matrix::operator()(const int index){
		return m_Data[index];
	}

	const double & Matrix::operator()(const int row, const int col) const{
		return m_Data[col * m_nRows + row];
	}

	double & Matrix::operator()(const int row, const int col){
		return m_Data[col * m_nRows + row];
	}

	Matrix Matrix::operator()(const vector<int> & rows, const vector<int> & cols) const{
		int i, j;
		Matrix retVal((int)rows.size(), (int)cols.size());

		for(i = 0; i < (int)rows.size(); i++){
			for(j = 0; j < (int)cols.size(); j++){
				retVal.set(i, j, m_Data[cols[j] * m_nRows + rows[i]]);
			}
		}

		return retVal;
	}

	Matrix Matrix::operator()(const int startrow, const int steprow, const int endrow, const int startcol, const int stepcol,
		const int endcol) const{

		int i, j;
		Matrix retVal(((endrow - startrow) / steprow) + 1, ((endcol - startcol) / stepcol) + 1);

		if(steprow > 0 && stepcol > 0){
			for(i = startrow; i <= endrow; i += steprow){
				for(j = startcol; j <= endcol; j += stepcol){
					retVal.set((i-startrow) / steprow, (j-startcol) / stepcol, m_Data[j * m_nRows + i]);
				}
			}
		}
		else if(steprow > 0 && stepcol < 0){
			for(i = startrow; i <= endrow; i += steprow){
				for(j = startcol; j >= endcol; j += stepcol){
					retVal.set((i-startrow) / steprow, (j-startcol) / stepcol, m_Data[j * m_nRows + i]);
				}
			}
		}
		else if(steprow < 0 && stepcol > 0){
			for(i = startrow; i >= endrow; i += steprow){
				for(j = startcol; j <= endcol; j += stepcol){
					retVal.set((i-startrow) / steprow, (j-startcol) / stepcol, m_Data[j * m_nRows + i]);
				}
			}
		}
		else{
			for(i = startrow; i >= endrow; i += steprow){
				for(j = startcol; j >= endcol; j += stepcol){
					retVal.set((i-startrow) / steprow, (j-startcol) / stepcol, m_Data[j * m_nRows + i]);
				}
			}
		}

		return retVal;
	}

	Matrix Matrix::operator()(const int startrow, const int endrow, const int startcol, const int endcol) const{
		int i, j;
		Matrix retVal(endrow - startrow + 1, endcol - startcol + 1);

		for(i = startrow; i <= endrow; i++){
			for(j = startcol; j <= endcol; j++){
				retVal.set(i-startrow, j-startcol, m_Data[j * m_nRows + i]);
			}
		}

		return retVal;
	}

	// Public Methods

	void Matrix::resize(const int rows, const int cols){
		// TODO: optimize
		int i, j, min_i, min_j;
		Matrix newMat(rows, cols);

		if(rows < m_nRows)
			min_i = rows;
		else min_i = m_nRows;

		if(cols < m_nCols)
			min_j = cols;
		else min_j = m_nCols;

		for(i = 0; i < min_i; i++){
			for(j = 0; j < min_j; j++){
				newMat.set(i, j, at(i, j));
			}
		}

		*this = newMat;
	}

	void Matrix::reserve(const int n){
		m_Data.reserve(n);
	}

	void Matrix::reserve(const int rows, const int cols){
		m_Data.reserve(rows * cols);
	}

	void Matrix::reshape(const int rows, const int cols){
		// TODO: check if dims agree.
		// TODO: check for matlab compliance in row-major vs column major.  May need to fix with a transpose (probably not).
		m_nRows = rows;
		m_nCols = cols;
	}

	void Matrix::clear(){
		m_Data.clear();
		m_nRows = 0;
		m_nCols = 0;
	}

	void Matrix::fill(const double fillval){
		int i, j;
		for(i = 0; i < m_nRows; i++){
			for(j = 0; j < m_nCols; j++){
				set(i, j, fillval);
			}
		}
	}

	double Matrix::det() const{
		// The determinant is computed from the triangular factors obtained by
		// Gaussian elimination [L,U] = lu(A)
		// s =  det(L)        % This is always +1 or -1 since L is the composition of a triangular matrix
		//                    % with unit diagonal and a permutation matrix.
		// det(A) = s*prod(diag(U))

		Matrix L, U;
		vector<int> piv;
		int i;
		double retVal;

		LU(L, U, piv);

		retVal = 1.0;
		for(i = 0; i < U.numRows(); i++){
			if(piv[i] != i)
				retVal *= -U(i, i);
			else
				retVal *= U(i, i);
		}

		return retVal;
	}

	Matrix Matrix::codedDet() const{
		// The determinant is computed from the triangular factors obtained by
		// Gaussian elimination [L,U] = lu(A)
		// s =  det(L)        % This is always +1 or -1 since L is the composition of a triangular matrix
		//                    % with unit diagonal and a permutation matrix.
		// det(A) = s*prod(diag(U))
		//
		// Determinant is returned as a 2 by 1 matrix with determinant = det(0) * 10.0**det(1), to avoid overflow / underflow

		Matrix L, U;
		vector<int> piv;
		int i;
		Matrix retVal(2,1);

		LU(L, U, piv);

		retVal(0) = 1.0;
		retVal(1) = 0.0;
		for(i = 0; i < U.numRows(); i++){
			if(piv[i] != i)
				retVal(0) *= -U(i, i);
			else
				retVal(0) *= U(i, i);

			if(retVal(0) == 0.0){
				retVal(1) = 0.0;
				break;
			}

			while( fabs(retVal(0)) < 1.0 ){
				retVal(0) *= 10.0;
				retVal(1) -= 1.0;
			}
			while( 10.0 <= fabs(retVal(0)) ){
				retVal(0) /= 10.0;
				retVal(1) += 1.0;
			}
		}

		return retVal;
	}

	int Matrix::rank() const{
		Matrix s = SVD(); // contains the singular vectors of this matrix.
		return round((s > ((double)std::max(m_nRows, m_nCols)) * eps((s.max())(0))).sumAll());
	}

	int Matrix::rank(const double tol) const{
		Matrix s = SVD(); // contains the singular vectors of this matrix.
		return round((s > tol).sumAll());
	}

	Matrix Matrix::inv() const{
		return mldivide(eye(m_nRows, m_nCols));
		// TODO: compute inverse more efficiently.  Try doc inv in MATLAB for a reference.
	}

	Matrix Matrix::mldivide(const Matrix & rhs) const{
		Matrix retVal;

		if(m_nRows == m_nCols){
			// Solve system via LU factorization and Forward- / Back-substition.
			long n, nrhs, lda, iPiv[m_nCols], ldb, info;
			n = m_nCols, nrhs = rhs.m_nCols; lda = m_nRows; ldb = rhs.m_nRows;
			Matrix tmpA = *this, tmpB = rhs;

			// Solve square system by LU factorization with partial pivoting, multiple right hand sides.
			dgesv_(&n, &nrhs, tmpA.m_Data.GetData(), &lda, iPiv, tmpB.m_Data.GetData(), &ldb, &info);
			retVal = tmpB;
		}
		else{
			// Least Squares via QR factorization w/ Column Pivoting to determine effective rank.
			int i;
			long m, n, lda, jpvt[m_nCols], lwork, info, k, ldb, ldc, nrhs;
			char side, trans, uplo, diag;
			double tau[std::min(m_nCols, m_nRows)], work_query;
			long effective_rank;

			Matrix tmpA = *this;
			Matrix tmpB = rhs;

			// QR with column pivoting using LAPACK routine DGEQP3
			{
				m = m_nRows; n = m_nCols; lda = m_nRows; lwork = -1L;
				// Mark all columns as free to swap by zeroing jpvt
				// TODO: speed zeroing of jpvt with something like calloc
				for(i = 0; i < m_nCols; i++){
					jpvt[i] = 0L;
				}

				// Run LAPACK workspace query and allocate workspace
				dgeqp3_(&m, &n, tmpA.m_Data.GetData(), &lda, jpvt, tau, &work_query, &lwork, &info);

				lwork = (long)work_query;
				double work[lwork];

				dgeqp3_(&m, &n, tmpA.m_Data.GetData(), &lda, jpvt, tau, work, &lwork, &info);
			}

			// Determine effective rank by checking diagonal elements via a relative tolerance of max(m, n)*eps w.r.t R(1,1)
			{
				if(tmpA(0,0) == 0.0)
					// effective_rank = 0L;
					// Therefore our solution contains all zeros.  Just return and subvert further computation:
					return zeros(m_nCols, tmpB.m_nCols);
				else{
					double tol = eps_d * std::max(m, n) * fabs(tmpA(0,0));
					for(i = 1; i < std::min(m,n); i++){
						if(fabs(tmpA(i,i)) < tol)
							break;
					}
					effective_rank = (long)i;
				}
			}

			// Compute basic solution.  Now we have A*P = Q*R, so compute the solution X = P*(R\(Q'*B)).

			// First, compute Q'*B via LAPACK routine xORMQR:
			{
				side = 'L'; trans = 'T';
				m = tmpB.m_nRows; n = tmpB.m_nCols;
				k = std::min(m_nCols, m_nRows);
				lda = m_nRows; ldc = m;

				// Run LAPACK workspace query and allocate workspace
				lwork = -1L;
				dormqr_(&side, &trans, &m, &n, &k, tmpA.m_Data.GetData(), &lda, tau, tmpB.m_Data.GetData(), &ldc, &work_query, &lwork, &info);

				lwork = (long)work_query;
				double work[lwork];

				dormqr_(&side, &trans, &m, &n, &k, tmpA.m_Data.GetData(), &lda, tau, tmpB.m_Data.GetData(), &ldc, work, &lwork, &info);
			}

			// Next, solve the triangular system R \ (Q'*B) with LAPACK routine DTRTRS, cropping matrix sizes to effective rank
			{
				uplo = 'U'; trans = 'N'; diag = 'N';
				n = m_nCols;
				nrhs = tmpB.m_nCols;
				/*lda = m_nRows; ...already set*/ ldb = tmpB.m_nRows;

				dtrtrs_(&uplo, &trans, &diag, &effective_rank, &nrhs, tmpA.m_Data.GetData(), &lda, tmpB.m_Data.GetData(), &ldb, &info);

				// Zero out the cropped part of tmpB
				tmpB.set((int)effective_rank, m_nCols - 1, 0, (int)nrhs - 1, 0.0);
				// tmpB now should hold (R\(Q'*B)).
			}

			// Apply the permutation matrix P to (R\(Q'*B)) and copy into retVal
			retVal = zeros((int)effective_rank + 1, (int)nrhs);
			for(i = 0; i < (int)effective_rank; i++)
				retVal.set((int)jpvt[i] - 1, (int)jpvt[i] - 1, 0, (int)nrhs - 1, tmpB(i, i, 0, (int)nrhs - 1));
		}
		return retVal;
	}

	Matrix Matrix::mrdivide(const Matrix & rhs) const{
		return rhs.transpose().mldivide(transpose()).transpose();
	}

	Matrix Matrix::times(const Matrix & rhs) const{
		int i, j;
		Matrix retVal(m_nRows, m_nCols);

		for(i = 0; i < m_nRows; i++){
			for(j = 0; j < m_nCols; j++){
				retVal.set(i, j, at(i, j) * rhs.at(i, j));
			}
		}

		return retVal;
	}

	Matrix Matrix::ldivide(const Matrix & rhs) const{
		int i, j;
		Matrix retVal(m_nRows, m_nCols);

		for(i = 0; i < m_nRows; i++){
			for(j = 0; j < m_nCols; j++){
				retVal.set(i, j, rhs.at(i, j) / at(i, j));
			}
		}

		return retVal;
	}

	Matrix Matrix::rdivide(const Matrix & rhs) const{
		int i, j;
		Matrix retVal(m_nRows, m_nCols);

		for(i = 0; i < m_nRows; i++){
			for(j = 0; j < m_nCols; j++){
				retVal.set(i, j, at(i, j) / rhs.at(i, j));
			}
		}

		return retVal;
	}

	Matrix Matrix::solveHomogeneous() const{
		// Solves the Homogeneous system of equations A*x=0 via the SVD, where A = *this.
		// Result minimizes norm(A*x - 0).
		Matrix A = *this, V(m_nCols, m_nCols);
		long m, n, lwork, info;
		char jobu = 'N', jobv = 'A';
		double work_query, s[std::min(m_nRows, m_nCols)];

		// Compute SVD via LAPACK routine DGESVD
		{
			m = m_nRows; n = m_nCols;

			// Run LAPACK workspace query and allocate workspace
			lwork = -1;
			dgesvd_(&jobu, &jobv, &m, &n, A.m_Data.GetData(), &m, s, 0, &m, V.m_Data.GetData(), &n,
				&work_query, &lwork, &info);

			lwork = (long)work_query;
			double work[lwork];

			dgesvd_(&jobu, &jobv, &m, &n, A.m_Data.GetData(), &m, s, 0, &m, V.m_Data.GetData(), &n,
				work, &lwork, &info);
		}

		// Extract solution x = V(end, :)'.  (V is really V')
		return V(m_nCols - 1, m_nCols - 1, 0, m_nCols - 1).transpose();
	}

	Matrix Matrix::null() const{
		// Computes an orthonormal basis for the nullspace via the SVD.
		// TODO: Actually determine effective rank appropriately.  See "doc null" in MATLAB
		Matrix A = *this, V(m_nCols, m_nCols), s(std::min(m_nRows, m_nCols), 1);
		long m, n, lwork, info;
		char jobu = 'N', jobv = 'A';
		double work_query;
		int effective_rank;

		// Compute SVD via LAPACK routine DGESVD
		{
			m = m_nRows; n = m_nCols;

			// Run LAPACK workspace query and allocate workspace
			lwork = -1;
			dgesvd_(&jobu, &jobv, &m, &n, A.m_Data.GetData(), &m, s.m_Data.GetData(), 0, &m, V.m_Data.GetData(), &n,
				&work_query, &lwork, &info);

			lwork = (long)work_query;
			double work[lwork];

			dgesvd_(&jobu, &jobv, &m, &n, A.m_Data.GetData(), &m, s.m_Data.GetData(), 0, &m, V.m_Data.GetData(), &n,
				work, &lwork, &info);
		}

		effective_rank = rank(s);

		// Extract the last rows of V (past the first effective_rank rows) as the basis, and transpose
		// to compute the result
		return V(effective_rank, m_nCols - 1, 0, m_nCols - 1).transpose();
	}

	Matrix Matrix::transpose() const{
		int i, j;
		Matrix retVal(m_nCols, m_nRows);

		for(i = 0; i < m_nRows; i++){
			for(j = 0; j < m_nCols; j++){
				retVal.set(j, i, at(i, j));
			}
		}

		return retVal;
	}

	Matrix Matrix::sum() const{
		int i, j, dim = 1;
		double tmp;
		Matrix retVal;

		if(isRowVector()) dim = 2;

		if(dim == 1){
			//retVal.SetData(vect<double>(m_nCols, 0.0));
			retVal.m_Data = vect<double>(m_nCols, 0.0);
			retVal.m_nRows = 1;
			retVal.m_nCols = m_nCols;

			for(j = 0; j < m_nCols; j++){
				tmp = 0.0;
				for(i = 0; i < m_nRows; i++){
					tmp += at(i, j);
				}
				retVal.set(0, j, tmp);
			}
		}
		else{
			//retVal.SetData(vect<double>(m_nRows, 0.0));
			retVal.m_Data = vect<double>(m_nRows, 0.0);
			retVal.m_nRows = m_nRows;
			retVal.m_nCols = 1;

			for(i = 0; i < m_nRows; i++){
				tmp = 0.0;
				for(j = 0; j < m_nCols; j++){
					tmp += at(i, j);
				}
				retVal.set(i, 0, tmp);
			}
		}

		return retVal;
	}

	Matrix Matrix::sum(const int dim) const{
		int i, j;
		double tmp;
		Matrix retVal;

		if(dim == 1){
			//retVal.SetData(vect<double>(m_nCols, 0.0));
			retVal.m_Data = vect<double>(m_nCols, 0.0);
			retVal.m_nRows = 1;
			retVal.m_nCols = m_nCols;

			for(j = 0; j < m_nCols; j++){
				tmp = 0.0;
				for(i = 0; i < m_nRows; i++){
					tmp += at(i, j);
				}
				retVal.set(0, j, tmp);
			}
		}
		else{
			//retVal.SetData(vect<double>(m_nRows, 0.0));
			retVal.m_Data = vect<double>(m_nRows, 0.0);
			retVal.m_nRows = m_nRows;
			retVal.m_nCols = 1;

			for(i = 0; i < m_nRows; i++){
				tmp = 0.0;
				for(j = 0; j < m_nCols; j++){
					tmp += at(i, j);
				}
				retVal.set(i, 0, tmp);
			}
		}

		return retVal;
	}

	double Matrix::sumAll() const{
		int i, j;
		double retVal = 0.0;

		for(i = 0; i < m_nRows; i++){
			for(j = 0; j < m_nCols; j++){
				retVal += at(i, j);
			}
		}

		return retVal;
	}

	Matrix Matrix::prod() const{
		int i, j, dim = 1;
		double tmp;
		Matrix retVal;

		if(isRowVector()) dim = 2;

		if(dim == 1){
			//retVal.SetData(vect<double>(m_nCols, 0.0));
			retVal.m_Data = vect<double>(m_nCols, 0.0);
			retVal.m_nRows = 1;
			retVal.m_nCols = m_nCols;

			for(j = 0; j < m_nCols; j++){
				tmp = 1.0;
				for(i = 0; i < m_nRows; i++){
					tmp *= at(i, j);
				}
				retVal.set(0, j, tmp);
			}
		}
		else{
			//retVal.SetData(vect<double>(m_nRows, 0.0));
			retVal.m_Data = vect<double>(m_nRows, 0.0);
			retVal.m_nRows = m_nRows;
			retVal.m_nCols = 1;

			for(i = 0; i < m_nRows; i++){
				tmp = 1.0;
				for(j = 0; j < m_nCols; j++){
					tmp *= at(i, j);
				}
				retVal.set(i, 0, tmp);
			}
		}

		return retVal;
	}

	Matrix Matrix::prod(const int dim) const{
		int i, j;
		double tmp;
		Matrix retVal;

		if(dim == 1){
			//retVal.SetData(vect<double>(m_nCols, 0.0));
			retVal.m_Data = vect<double>(m_nCols, 0.0);
			retVal.m_nRows = 1;
			retVal.m_nCols = m_nCols;

			for(j = 0; j < m_nCols; j++){
				tmp = 1.0;
				for(i = 0; i < m_nRows; i++){
					tmp *= at(i, j);
				}
				retVal.set(0, j, tmp);
			}
		}
		else{
			//retVal.SetData(vect<double>(m_nRows, 0.0));
			retVal.m_Data = vect<double>(m_nRows, 0.0);
			retVal.m_nRows = m_nRows;
			retVal.m_nCols = 1;

			for(i = 0; i < m_nRows; i++){
				tmp = 1.0;
				for(j = 0; j < m_nCols; j++){
					tmp *= at(i, j);
				}
				retVal.set(i, 0, tmp);
			}
		}

		return retVal;
	}

	double Matrix::prodAll() const{
		int i, j;
		double retVal = 1.0;

		for(i = 0; i < m_nRows; i++){
			for(j = 0; j < m_nCols; j++){
				retVal *= at(i, j);
			}
		}

		return retVal;
	}

	Matrix Matrix::mean() const{
		int i, j, dim = 1;
		double tmp;
		Matrix retVal;

		if(isRowVector()) dim = 2;

		if(dim == 1){
			retVal.m_Data = vect<double>(m_nCols, 0.0);
			retVal.m_nRows = 1;
			retVal.m_nCols = m_nCols;

			for(j = 0; j < m_nCols; j++){
				tmp = 0.0;
				for(i = 0; i < m_nRows; i++){
					tmp += at(i, j);
				}
				retVal.set(0, j, tmp / (double)m_nRows);
			}
		}
		else{
			retVal.m_Data = vect<double>(m_nRows, 0.0);
			retVal.m_nRows = m_nRows;
			retVal.m_nCols = 1;

			for(i = 0; i < m_nRows; i++){
				tmp = 0.0;
				for(j = 0; j < m_nCols; j++){
					tmp += at(i, j);
				}
				retVal.set(i, 0, tmp / (double)m_nCols);
			}
		}

		return retVal;
	}

	Matrix Matrix::mean(const int dim) const{
		int i, j;
		double tmp;
		Matrix retVal;

		if(dim == 1){
			retVal.m_Data = vect<double>(m_nCols, 0.0);
			retVal.m_nRows = 1;
			retVal.m_nCols = m_nCols;

			for(j = 0; j < m_nCols; j++){
				tmp = 0.0;
				for(i = 0; i < m_nRows; i++){
					tmp += at(i, j);
				}
				retVal.set(0, j, tmp / (double)m_nRows);
			}
		}
		else{
			retVal.m_Data = vect<double>(m_nRows, 0.0);
			retVal.m_nRows = m_nRows;
			retVal.m_nCols = 1;

			for(i = 0; i < m_nRows; i++){
				tmp = 0.0;
				for(j = 0; j < m_nCols; j++){
					tmp += at(i, j);
				}
				retVal.set(i, 0, tmp / (double)m_nCols);
			}
		}

		return retVal;
	}

	double Matrix::meanAll() const{
		int i, j;
		double retVal = 0.0;

		for(i = 0; i < m_nRows; i++){
			for(j = 0; j < m_nCols; j++){
				retVal += at(i, j);
			}
		}

		return retVal / (double)(m_nRows * m_nCols);
	}

	Matrix Matrix::var() const{
		// TODO: test
		int i, j, dim = 1;
		double tmp;
		Matrix retVal;

		if(isRowVector()) dim = 2;

		Matrix matMean(mean());

		if(dim == 1){
			retVal.m_Data = vect<double>(m_nCols, 0.0);
			retVal.m_nRows = 1;
			retVal.m_nCols = m_nCols;

			for(j = 0; j < m_nCols; j++){
				tmp = 0.0;
				for(i = 0; i < m_nRows; i++)
					tmp += (at(i, j) - matMean(j)) * (at(i, j) - matMean(j));
				retVal.set(0, j, tmp / (double)(m_nRows - 1)); // Best unbiased estimator.  2nd moment would omit the "- 1"
			}
		}
		else{
			retVal.m_Data = vect<double>(m_nRows, 0.0);
			retVal.m_nRows = m_nRows;
			retVal.m_nCols = 1;

			for(i = 0; i < m_nRows; i++){
				tmp = 0.0;
				for(j = 0; j < m_nCols; j++)
					tmp += (at(i, j) - matMean(i)) * (at(i, j) - matMean(i));
				retVal.set(i, 0, tmp / (double)(m_nCols - 1)); // Best unbiased estimator.  2nd moment would omit the "- 1"
			}
		}

		return retVal;
	}

	Matrix Matrix::var(const int dim) const{
		// TODO: test
		int i, j;
		double tmp;
		Matrix retVal;

		Matrix matMean(mean());

		if(dim == 1){
			retVal.m_Data = vect<double>(m_nCols, 0.0);
			retVal.m_nRows = 1;
			retVal.m_nCols = m_nCols;

			for(j = 0; j < m_nCols; j++){
				tmp = 0.0;
				for(i = 0; i < m_nRows; i++)
					tmp += (at(i, j) - matMean(j)) * (at(i, j) - matMean(j));
				retVal.set(0, j, tmp / (double)(m_nRows - 1)); // Best unbiased estimator.  2nd moment would omit the "- 1"
			}
		}
		else{
			retVal.m_Data = vect<double>(m_nRows, 0.0);
			retVal.m_nRows = m_nRows;
			retVal.m_nCols = 1;

			for(i = 0; i < m_nRows; i++){
				tmp = 0.0;
				for(j = 0; j < m_nCols; j++)
					tmp += (at(i, j) - matMean(i)) * (at(i, j) - matMean(i));
				retVal.set(i, 0, tmp / (double)(m_nCols - 1)); // Best unbiased estimator.  2nd moment would omit the "- 1"
			}
		}

		return retVal;
	}

	double Matrix::varAll() const{
		// TODO: test
		// TODO: See the wikipedia page for Variance for a formula we can exploit.  To make all var() and varAll() functions more efficient.  (Consider numerical issues first.)
		int i, j;
		double retVal = 0.0;

		double nMean = meanAll();

		for(i = 0; i < m_nRows; i++){
			for(j = 0; j < m_nCols; j++){
				retVal += (at(i, j) - nMean) * (at(i, j) - nMean);
			}
		}

		return retVal / (double)(m_nRows * m_nCols - 1); // Best unbiased estimator.  2nd moment would omit the "- 1"
	}

	Matrix Matrix::max() const{
		// Quick check for empty matrix
		if(isEmpty())
			return *this;

		Matrix retVal(1, m_nCols);
		int i, j;

		for(i = 0; i < m_nCols; i++){
			retVal(0, i) = at(0, i);
			for(j = 1; j < m_nRows; j++){
				if(at(j, i) > retVal(0, i))
					retVal(0, i) = at(j, i);
			}
		}

		return retVal;
	}

	Matrix Matrix::max(const Matrix & rhs) const{
		int i, j;
		Matrix retVal(m_nRows, m_nCols);

		for(i = 0; i < m_nRows; i++){
			for(j = 0; j < m_nCols; j++){
				retVal.set(i, j, std::max(at(i, j), rhs.at(i, j)));
			}
		}
		return retVal;
	}

	Matrix Matrix::max(const double rhs) const{
		int i, j;
		Matrix retVal(m_nRows, m_nCols);

		for(i = 0; i < m_nRows; i++){
			for(j = 0; j < m_nCols; j++){
				retVal.set(i, j, std::max(at(i, j), rhs));
			}
		}
		return retVal;
	}

	double Matrix::maxAll() const{
		int i, j;
		double retVal = at(0); // TODO: set to -inf once numeric limits is figured out

		for(i = 0; i < m_nRows; i++){
			for(j = 0; j < m_nCols; j++){
				if(at(i, j) > retVal)
					retVal = at(i, j);
			}
		}

		return retVal;
	}

	Matrix Matrix::min() const{
		// Quick Check for empty matrix
		if(isEmpty())
			return *this;

		Matrix retVal(1, m_nCols);
		int i, j;

		for(i = 0; i < m_nCols; i++){
			retVal(0, i) = at(0, i);
			for(j = 1; j < m_nRows; j++){
				if(at(j, i) < retVal(0, i))
					retVal(0, i) = at(j, i);
			}
		}

		return retVal;
	}

	Matrix Matrix::min(const Matrix & rhs) const{
		int i, j;
		Matrix retVal(m_nRows, m_nCols);

		for(i = 0; i < m_nRows; i++){
			for(j = 0; j < m_nCols; j++){
				retVal.set(i, j, std::min(at(i, j), rhs.at(i, j)));
			}
		}
		return retVal;
	}

	Matrix Matrix::min(const double rhs) const{
		int i, j;
		Matrix retVal(m_nRows, m_nCols);

		for(i = 0; i < m_nRows; i++){
			for(j = 0; j < m_nCols; j++){
				retVal.set(i, j, std::min(at(i, j), rhs));
			}
		}
		return retVal;
	}

	double Matrix::minAll() const{
		int i, j;
		double retVal = at(0); // TODO: set to inf once numeric limits is figured out

		for(i = 0; i < m_nRows; i++){
			for(j = 0; j < m_nCols; j++){
				if(at(i, j) < retVal)
					retVal = at(i, j);
			}
		}

		return retVal;
	}

	Matrix Matrix::diag() const{
		int nDim = std::min(m_nRows, m_nCols);
		Matrix retVal(nDim, 1);

		for(int i = 0; i < nDim; i++)
			retVal(i) = at(i, i);

		return retVal;
	}

	double Matrix::trace() const{
		int nDim = std::min(m_nRows, m_nCols);
		double retVal = 0.0;

		for(int i = 0; i < nDim; i++)
			retVal += at(i, i);

		return retVal;
	}

	Matrix Matrix::abs() const{
		int i, j;
		Matrix retVal(m_nRows, m_nCols);
		for(i = 0; i < m_nRows; i++){
			for(j = 0; j < m_nCols; j++){
				retVal.set(i, j, fabs(at(i, j)));
			}
		}
		return retVal;
	}

	double Matrix::dot(const Matrix & rhs) const{
		// TODO: check if dimensions agree (and they are vectors)
		int nSize = std::max(m_nCols, m_nRows);
		double retVal = 0.0;

		for(int i = 0; i < nSize; i++)
			retVal += at(i)*rhs.at(i);

		return retVal;
	}

	Matrix Matrix::cross(const Matrix & rhs) const{
		// TODO: check that *this and rhs are 3-vectors
		Matrix retVal(3, 1);
		retVal(0) = at(1) * rhs(2) - at(2) * rhs(1);
		retVal(1) = at(2) * rhs(0) - at(0) * rhs(2);
		retVal(2) = at(0) * rhs(1) - at(1) * rhs(0);
		return retVal;
	}

	double Matrix::angleBetween(const Matrix & rhs) const{
		// TODO: check that *this and rhs are vectors
		double normal = norm();
		double rhsNormal = rhs.norm();

		if(doubleEquals(normal, 0.0) || doubleEquals(rhsNormal, 0.0))
			return 0.0;
		return acos(dot(rhs) / (normal * rhsNormal));
	}

	double Matrix::norm(const PNormType normType) const{
		double retVal = 0.0;

		if(normType == PNORM_ONE){
			if(isVector())
				retVal = abs().sumAll(); // sum(abs(A))
			else
				retVal = abs().sum().maxAll(); // largest column sum of absolute values of A.
		}
		else if(normType == PNORM_TWO){
			if(isVector())
				retVal = sqrt(dot(*this)); // sqrt(A.dot(A));
			else
				retVal = (SVD())(0); // largest singular value of A
		}
		else if(normType == PNORM_FROBENIUS){
			if(isVector())
				retVal = sqrt(dot(*this)); // sqrt(sumsq(abs(a)))
			else
				retVal = sqrt((transpose() * (*this)).diag().sumAll()); // sqrt(sum(diag(A' * A)))
		}
		else if(normType == PNORM_INF){
			if(isVector())
				retVal = abs().maxAll(); // max(abs(A));
			else
				retVal = abs().sum(2).maxAll(); // largest row sum of absolute values of A.
		}
		else if(normType == PNORM_NEG_INF){
			retVal = abs().minAll(); // min(abs(A))
		}
		else{
			// TODO: should not reach here.  throw exception.
		}

		return retVal;
	}

	double Matrix::norm(const double p) const{
		// Vector p-norm
		// TODO: check for NaN...?
		return pow((abs()^p).sumAll(), 1.0 / p);
	}

	Matrix Matrix::tril() const{
		int i, j;
		Matrix retVal(m_nRows, m_nCols);

		for(j = 0; j < m_nCols; j++){
			for(i = j; i < m_nRows; i++){
				retVal.set(i, j, at(i, j));
			}
		}

		return retVal;
	}

	Matrix Matrix::triu() const{
		int i, j;
		Matrix retVal(m_nRows, m_nCols);

		for(i = 0; i < m_nRows; i++){
			for(j = i; j < m_nCols; j++){
				retVal.set(i, j, at(i, j));
			}
		}

		return retVal;
	}

	void Matrix::LU(Matrix & L, Matrix & U) const{
		int i, min_dimension = std::min(m_nRows, m_nCols);
		long m, n, lda, ipiv[min_dimension], info;
		Matrix A(*this), L_tmp;

		m = m_nRows; n = m_nCols; lda = m_nRows;

		// LU factorization of a general M-by-N matrix A using partial pivoting with row interchanges.
		dgetrf_(&m, &n, A.m_Data.GetData(), &lda, ipiv, &info);

		// Extract L and U from A
		L_tmp = A.tril();
		for(i = 0; i < min_dimension; i++)
			L_tmp.set(i, i, 1.0);
		U = A.triu();

		// Permute matrix L with ipiv s.t. A = L*U, where L is now a permutation of a lower triangular matrix
		for(i = 0; i < min_dimension; i++)
			L.set((int)ipiv[i] - 1, (int)ipiv[i] - 1, 0, min_dimension - 1, L_tmp(i, i, 0, min_dimension - 1));
	}

	void Matrix::LU(Matrix & L, Matrix & U, Matrix & P) const{
		int i, min_dimension = std::min(m_nRows, m_nCols);
		long m, n, lda, ipiv[min_dimension], info;
		Matrix A(*this);

		m = m_nRows; n = m_nCols; lda = m_nRows;

		// LU factorization of a general M-by-N matrix A using partial pivoting with row interchanges.
		dgetrf_(&m, &n, A.m_Data.GetData(), &lda, ipiv, &info);

		// Extract L and U from A
		L = A.tril();
		for(i = 0; i < min_dimension; i++)
			L.set(i, i, 1.0);
		U = A.triu();

		// Construct permutation matrix P from ipiv
		P = zeros(m_nRows, m_nRows);
		for(i = 0; i < min_dimension; i++)
			P.set(i, (int)ipiv[i] - 1, 1.0);
	}

	void Matrix::LU(Matrix & L, Matrix & U, vector<int> & piv) const{
		int i, min_dimension = std::min(m_nRows, m_nCols);
		long m, n, lda, ipiv[min_dimension], info;
		Matrix A(*this);

		m = m_nRows; n = m_nCols; lda = m_nRows;

		// LU factorization of a general M-by-N matrix A using partial pivoting with row interchanges.
		dgetrf_(&m, &n, A.m_Data.GetData(), &lda, ipiv, &info);

		// Extract L and U from A
		L = A.tril();
		for(i = 0; i < min_dimension; i++)
			L.set(i, i, 1.0);
		U = A.triu();

		// Copy data from ipiv to piv
		piv.clear();
		piv.reserve(min_dimension);
		for(i = 0; i < min_dimension; i++)
			piv.push_back(ipiv[i] - 1);
	}

	void Matrix::QR(Matrix & Q, Matrix & R) const{
		// QR factorization without pivoting
		long m, n, lwork, info, k = (long)std::min(m_nRows, m_nCols);
		double tau[k], work_query;

		Matrix tmpA = *this;

		// QR using LAPACK routine DGEQRF
		{
			m = m_nRows; n = m_nCols;

			// Run LAPACK workspace query and allocate workspace
			lwork = -1L;
			dgeqrf_(&m, &n, tmpA.m_Data.GetData(), &m, tau, &work_query, &lwork, &info);

			lwork = (long)work_query;
			double work[lwork];

			dgeqrf_(&m, &n, tmpA.m_Data.GetData(), &m, tau, work, &lwork, &info);
		}

		// Extract R
		R = tmpA.triu();

		// Extract / compute Q via LAPACK routine DORGQR
		{
			Q = Matrix(m_nRows, m_nRows);
			Q.set(0, m_nRows-1, 0, (int)k - 1, tmpA(0, m_nRows-1, 0, (int)k - 1));

			// Run LAPACK workspace query and allocate workspace
			lwork = -1L;
			dorgqr_(&m, &m, &k, Q.m_Data.GetData(), &m, tau, &work_query, &lwork, &info);

			lwork = (long)work_query;
			double work[lwork];

			dorgqr_(&m, &m, &k, Q.m_Data.GetData(), &m, tau, work, &lwork, &info);
		}
	}

	void Matrix::QR(Matrix & Q, Matrix & R, Matrix & P) const{
		// QR factorization w/ Column Pivoting
		int i;
		long m, n, jpvt[m_nCols], lwork, info, k = (long)std::min(m_nRows, m_nCols);
		double tau[k], work_query;

		Matrix tmpA = *this;

		// QR with column pivoting using LAPACK routine DGEQP3
		{
			m = m_nRows; n = m_nCols; lwork = -1L;
			// Mark all columns as free to swap by zeroing jpvt
			// TODO: speed zeroing of jpvt with something like calloc
			for(i = 0; i < m_nCols; i++){
				jpvt[i] = 0L;
			}

			// Run LAPACK workspace query and allocate workspace
			dgeqp3_(&m, &n, tmpA.m_Data.GetData(), &m, jpvt, tau, &work_query, &lwork, &info);

			lwork = (long)work_query;
			double work[lwork];

			dgeqp3_(&m, &n, tmpA.m_Data.GetData(), &m, jpvt, tau, work, &lwork, &info);
		}

		// Extract R
		R = tmpA.triu();

		// Extract / compute Q via LAPACK routine DORGQR
		{
			Q = Matrix(m_nRows, m_nRows);
			Q.set(0, m_nRows-1, 0, (int)k - 1, tmpA(0, m_nRows-1, 0, (int)k - 1));

			// Run LAPACK workspace query and allocate workspace
			lwork = -1L;
			dorgqr_(&m, &m, &k, Q.m_Data.GetData(), &m, tau, &work_query, &lwork, &info);

			lwork = (long)work_query;
			double work[lwork];

			dorgqr_(&m, &m, &k, Q.m_Data.GetData(), &m, tau, work, &lwork, &info);
		}

		// Compute permutation matrix P from jpvt
		P = zeros(m_nCols, m_nCols);
		for(i = 0; i < m_nCols; i++)
			P((int)jpvt[i] - 1, i) = 1.0;
	}

	Matrix Matrix::SVD() const{
		// Returns a vector of the Singular Values of this matrix.
		Matrix A = *this, retVal(std::min(m_nRows, m_nCols), 1);
		long m, n, lwork, info;
		char job = 'N';
		double work_query;

		// Compute SVD via LAPACK routine DGESVD
		{
			m = m_nRows; n = m_nCols;

			// Run LAPACK workspace query and allocate workspace
			lwork = -1;
			dgesvd_(&job, &job, &m, &n, A.m_Data.GetData(), &m, retVal.m_Data.GetData(), 0, &m, 0, &n,
				&work_query, &lwork, &info);

			lwork = (long)work_query;
			double work[lwork];

			dgesvd_(&job, &job, &m, &n, A.m_Data.GetData(), &m, retVal.m_Data.GetData(), 0, &m, 0, &n,
				work, &lwork, &info);
		}

		return retVal;
	}

	void Matrix::SVD(Matrix & U, Matrix & S, Matrix & V) const{
		// Performs a SVD decomposition of A (= *this) s.t. A = U*S*V'.
		Matrix A = *this;
		long m, n, lwork, info;
		char job = 'A';
		double work_query, s[std::min(m_nRows, m_nCols)];
		int i;

		// Compute SVD via LAPACK routine DGESVD
		{
			U = Matrix(m_nRows, m_nRows);
			V = Matrix(m_nCols, m_nCols);
			m = m_nRows; n = m_nCols;

			// Run LAPACK workspace query and allocate workspace
			lwork = -1;
			dgesvd_(&job, &job, &m, &n, A.m_Data.GetData(), &m, s, U.m_Data.GetData(), &m, V.m_Data.GetData(), &n,
				&work_query, &lwork, &info);

			lwork = (long)work_query;
			double work[lwork];

			dgesvd_(&job, &job, &m, &n, A.m_Data.GetData(), &m, s, U.m_Data.GetData(), &m, V.m_Data.GetData(), &n,
				work, &lwork, &info);
		}

		// Convert output from V' to V.
		V = V.transpose();

		// Construct matrix S from singular value array s.
		S = zeros(m_nRows, m_nCols);
		for(i = 0; i < std::min(m_nRows, m_nCols); i++)
			S(i,i) = s[i];
	}

	bool Matrix::isColVector() const{
		return m_nCols == 1 && !isEmpty();
	}

	bool Matrix::isRowVector() const{
		return m_nRows == 1 && !isEmpty();
	}

	bool Matrix::isVector() const{
		return (m_nCols == 1 || m_nRows == 1) && !isEmpty();
	}

	bool Matrix::isEmpty() const{
		return m_nCols == 0 || m_nRows == 0;
	}

	Matrix & Matrix::hcat(const Matrix & rhs){
		//TODO: check sanity against concatinating with *this
		resize(m_nRows, m_nCols + rhs.numCols());
		set(0, m_nRows-1, m_nCols - rhs.numCols(), m_nCols - 1, rhs);
		return *this;
	}

	Matrix & Matrix::vcat(const Matrix & rhs){
		//TODO: check sanity against concatinating with *this
		resize(m_nRows + rhs.numRows(), m_nCols);
		set(m_nRows - rhs.numRows(), m_nRows - 1, 0, m_nCols - 1, rhs);
		return *this;
	}

	Matrix Matrix::pflat() const{
		Matrix retVal(m_nRows, m_nCols, 1.0);

		for(int i = 0; i < m_nCols; i++){
			if(at(m_nRows - 1, i) != 0.0){
				for(int j = 0; j < m_nRows - 1; j++)
					retVal.set(j, i, at(j, i) / at(m_nRows-1, i));
			}
			else{
				for(int j = 0; j < m_nRows; j++)
					retVal.set(j, i, at(j, i));
			}
		}

		return retVal;
	}

	Matrix Matrix::conv(const Matrix & rhs) const{
		// TODO: implement in terms of filter() for efficiency.  try "type filter" and "type conv".
		// TODO: speed innermost loops' index computations
		int nSizeLHS = numElements();
		int nSizeRHS = rhs.numElements();
		int nSizeResult = nSizeLHS + nSizeRHS - 1;
		int index;
		Matrix retVal(1, nSizeResult, 0.0);

		// use the smaller one as the filter.
		if(nSizeLHS < nSizeRHS){
			for(int i = 0; i < nSizeResult; i++){
				// compute retVal(i)
				for(int j = 1; j <= nSizeLHS; j++){
					index = i - nSizeLHS + j;
					if(index >= 0 && index < nSizeRHS) // use zero to pad
						retVal(i) += rhs(index) * at(nSizeLHS - j);
				}
			}
		}
		else{
			for(int i = 0; i < nSizeResult; i++){
				// compute retVal(i)
				for(int j = 1; j <= nSizeRHS; j++){
					index = i - nSizeRHS + j;
					if(index >= 0 && index < nSizeLHS) // use zero to pad
						retVal(i) += at(index) * rhs(nSizeRHS - j);
				}
			}
		}

		return retVal;
	}

	Matrix Matrix::conv2(const Matrix & rhs, const ConvolutionType type) const{
		// TODO: make more efficient if possible (move away from simple spatial convolution).
		// TODO: Speed the innermost loops' index computation.

		Matrix retVal;

		if(type==CONV_FULL){
			int nRowIndex, nColIndex;
			retVal = Matrix(m_nRows + rhs.m_nRows - 1, m_nCols + rhs.m_nCols - 1, 0.0);

			for(int i = 0; i < retVal.m_nRows; i++){
				for(int j = 0; j < retVal.m_nCols; j++){
					// compute retVal(i, j)
					for(int p = 1; p <= rhs.m_nRows; p++){
						for(int q = 1; q <= rhs.m_nCols; q++){
							nRowIndex = i - rhs.m_nRows + p;
							nColIndex = j - rhs.m_nCols + q;
							if(nRowIndex >= 0 && nRowIndex < m_nRows && nColIndex >= 0 && nColIndex < m_nCols) // use zero to pad
								retVal(i, j) += at(nRowIndex, nColIndex) * rhs(rhs.m_nRows - p, rhs.m_nCols - q);
						}
					}
				}
			}
		}
		else if(type == CONV_SAME){
			int nRowIndex, nColIndex;
			retVal = Matrix(m_nRows, m_nCols, 0.0);

			for(int i = 0; i < retVal.m_nRows; i++){
				for(int j = 0; j < retVal.m_nCols; j++){
					// compute retVal(i, j)
					for(int p = 1; p <= rhs.m_nRows; p++){
						for(int q = 1; q <= rhs.m_nCols; q++){
							nRowIndex = i - (rhs.m_nRows + 1) / 2 + p;
							nColIndex = j - (rhs.m_nCols + 1) / 2 + q;
							if(nRowIndex >= 0 && nRowIndex < m_nRows && nColIndex >= 0 && nColIndex < m_nCols) // use zero to pad
								retVal(i, j) += at(nRowIndex, nColIndex) * rhs(rhs.m_nRows - p, rhs.m_nCols - q);
						}
					}
				}
			}
		}
		else if(type == CONV_VALID){
			int nRowIndex, nColIndex;
			retVal = Matrix(m_nRows - rhs.m_nRows + 1, m_nCols - rhs.m_nCols + 1, 0.0);

			for(int i = 0; i < retVal.m_nRows; i++){
				for(int j = 0; j < retVal.m_nCols; j++){
					// compute retVal(i, j)
					for(int p = 1; p <= rhs.m_nRows; p++){
						for(int q = 1; q <= rhs.m_nCols; q++){
							nRowIndex = i - 1 + p;
							nColIndex = j - 1 + q;
							if(nRowIndex >= 0 && nRowIndex < m_nRows && nColIndex >= 0 && nColIndex < m_nCols) // use zero to pad
								retVal(i, j) += at(nRowIndex, nColIndex) * rhs(rhs.m_nRows - p, rhs.m_nCols - q);
						}
					}
				}
			}
		}
		else{ // TODO: invalid type.  should not happen.  throw exception.
		}
		return retVal;
	}

	Matrix  Matrix::filter2(const Matrix & rhs, const ConvolutionType type) const{
		// rotate the filter rhs 180 degrees, and call conv2.
		return conv2(rhs(rhs.m_nRows - 1, -1, 0, rhs.m_nCols - 1, -1, 0), type);
	}

	string Matrix::toString() const{
		ostringstream o;
		int i, j;

		o << "[";
		for(i = 0; i < m_nRows; i++){
			for(j = 0; j < m_nCols; j++){
				o << at(i, j);
				if(j != (m_nCols - 1))
					o << ",\t";
			}
			if(i != (m_nRows - 1))
				o << ";" << endl << " ";
		}
		o << "]";

		return o.str();
	}

	string Matrix::toInfoString() const{
		ostringstream o;
		o << "Matrix: dims(" << m_nRows << " by " << m_nCols << ")";
		return o.str();
	}

	// Public Static Methods

	Matrix Matrix::eye(const int size){
		int i;
		Matrix retVal(size, size);

		for(i = 0; i < size; i++){
			retVal.set(i, i, 1.0);
		}

		return retVal;
	}

	Matrix Matrix::eye(const int rows, const int cols){
		int i, minsize;
		Matrix retVal(rows, cols);

		if(rows < cols) minsize = rows;
		else minsize = cols;

		for(i = 0; i < minsize; i++){
			retVal.set(i, i, 1.0);
		}

		return retVal;
	}

	Matrix Matrix::zeros(const int rows, const int cols){
		return Matrix(rows, cols);
	}

	Matrix Matrix::ones(const int rows, const int cols){
		return Matrix(rows, cols, 1.0);
	}

	Matrix Matrix::diag(const int size, const double val){
		int i;
		Matrix retVal(size, size);

		for(i = 0; i < size; i++){
			retVal.set(i, i, val);
		}

		return retVal;
	}

	Matrix Matrix::diag(const vector<double> & vals){
		int i, size = vals.size();
		Matrix retVal(size, size);

		for(i = 0; i < size; i++){
			retVal.set(i, i, vals[i]);
		}

		return retVal;
	}

	Matrix Matrix::exp(const Matrix & rhs){
		Matrix retVal(rhs.m_nRows, rhs.m_nCols);
		for(int i = 0; i < rhs.m_nRows * rhs.m_nCols; i++){
			retVal(i) = std::exp(rhs(i));
		}
		return retVal;
	}

	Matrix Matrix::fspecial(const FSpecialType type, const Matrix & size, const double param1){
		if(type == FSPECIAL_GAUSSIAN){
			int nRows = 0;
			int nCols = 0;
			double sigma = 0.5;

			if(size.numElements() == 1)
				nRows = nCols = round(size(0));
			else if(size.numElements() == 2){
				nRows = round(size(0));
				nCols = round(size(1));
			}
			else{
				// TODO: Throw an exception
			}

			sigma = param1;

			//meshgrid here:
			Matrix x(nRows, nCols);
			Matrix y(nRows, nCols);
			for(int i = 0; i < nRows; i++){
				for(int j = 0; j < nCols; j++){
					x(i,j) = j;
					y(i,j) = i;
				}
			}
			x = x - (nCols - 1) / 2.0;
			y = y - (nRows - 1) / 2.0;
			Matrix gauss = exp( -((x^2) + (y^2)) / (2.0*sigma*sigma) );
			return gauss / gauss.sumAll();
		}
		else{
			// TODO: implement.
			return Matrix();
		}
	}

	Matrix Matrix::rot2D(const double theta){
		Matrix retVal = eye(3);

		double c = cos(theta);
		double s = sin(theta);
		retVal(0, 0) = c;
		retVal(0, 1) = -s;
		retVal(1, 0) = s;
		retVal(1, 1) = c;

		return retVal;
	}

	Matrix Matrix::trans2D(const double x, const double y){
		Matrix retVal = eye(3);

		retVal(0,2) = x;
		retVal(1,2) = y;

		return retVal;
	}

	Matrix Matrix::rotx3D(const double theta){
		Matrix retVal = eye(4);

		double c = cos(theta);
		double s = sin(theta);
		retVal(1, 1) = c;
		retVal(1, 2) = -s;
		retVal(2, 1) = s;
		retVal(2, 2) = c;

		return retVal;
	}

	Matrix Matrix::roty3D(const double theta){
		Matrix retVal = eye(4);

		double c = cos(theta);
		double s = sin(theta);
		retVal(0, 0) = c;
		retVal(0, 2) = s;
		retVal(2, 0) = -s;
		retVal(2, 2) = c;

		return retVal;
	}

	Matrix Matrix::rotz3D(const double theta){
		Matrix retVal = eye(4);

		double c = cos(theta);
		double s = sin(theta);
		retVal(0, 0) = c;
		retVal(0, 1) = -s;
		retVal(1, 0) = s;
		retVal(1, 1) = c;

		return retVal;
	}

	Matrix Matrix::rot3D(const Matrix & t){
		// Calculates 3x3 rotation matrix from axis-angle vector t
		Matrix retVal;
		double theta = t.norm();

		if(doubleEquals(theta, 0.0))
			retVal = eye(3);
		else{
			retVal = skewSym(t / theta);
			retVal = eye(3) + sin(theta) * retVal + (1.0 - cos(theta)) * (retVal * retVal);
		}

		return retVal;
	}

	Matrix Matrix::rot3D(const double theta, const Matrix & t){
		// Calculates 3x3 rotation matrix from the angle and unit axis-vector via the Rodrigues formula
		Matrix retVal(skewSym(t));
		retVal = eye(3) + sin(theta) * retVal + (1.0 - cos(theta)) * (retVal * retVal);

		return retVal;
	}

	Matrix Matrix::applyRot3D(const Matrix & t, const Matrix & x){
		Matrix retVal;
		double theta = t.norm();

		if(doubleEquals(theta, 0.0))
			retVal = x;
		else{
			retVal = t / theta;
			retVal = x + sin(theta) * retVal.cross(x) + (1.0 - cos(theta)) * retVal.cross(retVal.cross(x));
		}
		return retVal;
	}

	Matrix Matrix::applyRot3D(const double theta, const Matrix & t, const Matrix & x){
		// Applies the 3x3 rotation matrix from the angle and unit axis-vector via the Rodrigues formula, without computing the rotation matrix explicitly
		if(doubleEquals(theta, 0.0))
			return x;
		return x + sin(theta) * t.cross(x) + (1.0 - cos(theta)) * t.cross(t.cross(x));
	}

	Matrix Matrix::axisAngleFromRot3D(const Matrix & R){
		// Computes axis-angle vector t from R
		Matrix v; double theta;
		Matrix::axisAngleFromRot3D(v, theta, R);
		return theta * v;
	}

	void Matrix::axisAngleFromRot3D(Matrix & v, double & theta, const Matrix & R){
		// Computes axis vector v and angle t from R
		Matrix v_hat(3, 1);
		double two_cos_theta, two_sin_theta;

		v = (R - eye(3)).solveHomogeneous(); // TODO: better way to find the unit eigenvector?  Currently does a SVD to compute an eigenvalue from the characteristic equation....

		v_hat(0) = R(2, 1) - R(1, 2);
		v_hat(1) = R(0, 2) - R(2, 0);
		v_hat(2) = R(1, 0) - R(0, 1);

		two_cos_theta = (R.trace() - 1.0);
		two_sin_theta = v.dot(v_hat);

		theta = atan2(two_sin_theta, two_cos_theta);
	}

	Matrix Matrix::scale3D(const double sx, const double sy, const double sz){
		Matrix retVal = eye(4);

		retVal(0, 0) = sx;
		retVal(1, 1) = sy;
		retVal(2, 2) = sz;

		return retVal;
	}

	Matrix Matrix::trans3D(const double x, const double y, const double z){
		Matrix retVal = eye(4);

		retVal(0, 3) = x;
		retVal(1, 3) = y;
		retVal(2, 3) = z;

		return retVal;
	}

	Matrix Matrix::skewSym(const Matrix & t){
		// Given a 3-vector t, return the corresponding 3x3 skew-symmetric matrix.
		// TODO: implement for non-3x3 matrices if it generalizes (which it probably does).
		Matrix retVal(3,3, 0.0);

		retVal(0, 1) = -t(2);
		retVal(0, 2) = t(1);
		retVal(1, 0) = t(2);
		retVal(1, 2) = -t(0);
		retVal(2, 0) = -t(1);
		retVal(2, 1) = t(0);

		return retVal;
	}

	// Private Methods

	int Matrix::rank(const Matrix & s) const{
		return round((s > ((double)std::max(m_nRows, m_nCols)) * eps((s.max())(0))).sumAll());
	}

	int Matrix::rank(const Matrix & s, const double tol) const{
		return round((s > tol).sumAll());
	}

	// Left hand binary operators

	Matrix operator+(const double lhs, const Matrix & rhs){
		return rhs + lhs;
	}

	Matrix operator-(const double lhs, const Matrix & rhs){
		// TODO: make more efficient
		Matrix mLhs(rhs.numRows(), rhs.numCols(), lhs);
		return mLhs - rhs;
	}

	Matrix operator*(const double lhs, const Matrix & rhs){
		return rhs * lhs;
	}

	Matrix operator/(const double lhs, const Matrix & rhs){
		// TODO: make more efficient
		Matrix mLhs(rhs.numRows(), rhs.numCols(), lhs);
		return mLhs.rdivide(rhs);
	}

	Matrix operator^(const double lhs, const Matrix & rhs){
		// TODO: make more efficient
		Matrix mLhs(rhs.numRows(), rhs.numCols(), lhs);
		return mLhs ^ rhs;
	}

	Matrix operator==(const double lhs, const Matrix & rhs){
		return rhs == lhs;
	}

	Matrix operator!=(const double lhs, const Matrix & rhs){
		return rhs != lhs;
	}

	Matrix operator>=(const double lhs, const Matrix & rhs){
		return rhs <= lhs;
	}

	Matrix operator<=(const double lhs, const Matrix & rhs){
		return rhs >= lhs;
	}

	Matrix operator>(const double lhs, const Matrix & rhs){
		return rhs < lhs;
	}

	Matrix operator<(const double lhs, const Matrix & rhs){
		return rhs > lhs;
	}
}

#endif
