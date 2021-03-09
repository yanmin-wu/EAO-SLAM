#ifndef __LAPACK_DECLARATIONS_H
#define __LAPACK_DECLARATIONS_H

extern "C" {
	// Linear solvers
	// ===============================================================================================

	// Solution of square linear system(s) AX=B, by LU decomp.
	void dgesv_(long *n, long *nrhs, double *A, long *lda, long *iPiv, double *B, long *ldb, long *info);

	// Solves triangular systems AX=B or A**T * X = B
	void dtrtrs_(char *uplo, char *trans, char *diag, long *n, long *nrhs, double *A, long *lda, double *B, long *ldb, long *info);

	// Decompositions
	// ===============================================================================================

	// LU factorization of a general M-by-N matrix A using partial pivoting with row interchanges.
	void dgetrf_(long *m, long *n, double *a, long *lda, long *ipiv, long *info);

	// QR factorization, without pivoting.
	void dgeqrf_(long *m, long *n, double *A, long *lda, double *tau, double *work, long *lwork, long *info);

	// QR With Column Pivoting
	void dgeqp3_(long *m, long *n, double *A, long *lda, long *jpvt, double *tau, double *work, long *lwork, long *info);

	// SVD
	void dgesvd_(char *jobu, char *jobvt, long *m, long *n, double *A, long *lda, double *s, double *U, long *ldu, double *VT, long *ldvt,
		double *work, long *lwork, long *info);


	// Misc. Helper routines
	// ===============================================================================================

	// Fast orthogonal matrix w/ real matrix multiplication routine
	void dormqr_(char *side, char *trans, long *m, long *n, long *k, double *A, long *lda, double *tau, double *C, long *ldc, double *work,
		long *lwork, long *info);

	// Constructs orthogonal matrix from householder transformation vectors / scalars, as returned by LAPACK qr factorization routines
	void dorgqr_(long *m, long *n, long *k, double *A, long *lda, double *tau, double *work, long *lwork, long *info);
} 

#endif
