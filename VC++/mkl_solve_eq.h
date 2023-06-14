#pragma once
#include "mkl_pardiso.h"
//#include "mkl_types.h"
//#include "mkl_lapacke.h"
#include "./Eigen/SparseCore"
using namespace Eigen;
typedef Eigen::SparseMatrix<double, RowMajor> SpMat;

class mkl_eq_solver
{
public:
	MKL_INT n;
	MKL_INT *ia;
	MKL_INT *ja;
	double *a;
	void *pt[64];
	MKL_INT mtype;       /* Real symmetric positive definite */
	MKL_INT iparm[64];
	MKL_INT maxfct, mnum, phase, error, msglvl;
	double ddum;          /* Double dummy */
	MKL_INT idum;         /* Integer dummy. */
	MKL_INT nrhs;     /* Number of right hand sides. */

	double *b;
	double *x;

	mkl_eq_solver();
	mkl_eq_solver(SpMat matrixAMM);
	void choleskyfactorize();
	void iterativeSolve(double *bfield);

};


