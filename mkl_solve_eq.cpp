
#pragma once
#include "mkl_solve_eq.h"



mkl_eq_solver::mkl_eq_solver()
{
}
// initilize the solver from a symmetric definite matrix
mkl_eq_solver::mkl_eq_solver(SpMat matrixAMM)
{
	n = matrixAMM.rows();//rank
	//sparse indexes
	ia = new MKL_INT[matrixAMM.rows() + 1];
	ja = new MKL_INT[(matrixAMM.rows() + matrixAMM.nonZeros()) / 2];
	//sparse values
	a = new double[(matrixAMM.rows() + matrixAMM.nonZeros()) / 2];
	int count = 0;
	int i;
	//store the eigen sparse matrix to MKL style
	for (i = 0; i < matrixAMM.outerSize(); ++i)
	{
		ia[i] = count + 1;

		for (Eigen::SparseMatrix<double, RowMajor>::InnerIterator it(matrixAMM, i); it; ++it)
		{
			if (it.row() <= it.col())
			{
				ja[count] = it.col() + 1;
				a[count] = it.value();
				count++;
			}

		}
	}

	ia[matrixAMM.rows()] = count + 1;
	/* Pardiso control parameters. */
	//pointer pt
	for (i = 0; i < 64; i++)
	{
		pt[i] = 0;
	}
	mtype = 2;       /* Real symmetric positive definite */
	for (i = 0; i < 64; i++)
	{
		iparm[i] = 0;
	}


	/* Pardiso control parameters and Auxiliary variables. */
	error = 0;        /* Initialize error flag */
	maxfct = 1;       /* Maximum number of numerical factorizations. */
	mnum = 1;         /* Which factorization to use. */
	msglvl = 0;       /* Print statistical information in file */
	nrhs = 3;     /* Number of right hand sides. */

	b = new double[nrhs * matrixAMM.rows()]; /* store right hand matrices */
	x = new double[nrhs * matrixAMM.rows()]; /* store solution matrices */
	iparm[0] = 1;         /* No solver default */
	iparm[1] = 2;         /* Fill-in reordering from METIS */
	iparm[3] = 0;         /* No iterative-direct algorithm */
	iparm[4] = 0;         /* No user fill-in reducing permutation */
	iparm[5] = 0;         /* Write solution into x */
	iparm[6] = 0;         /* Not in use */
	iparm[7] = 2;         /* Max numbers of iterative refinement steps */
	iparm[8] = 0;         /* Not in use */
	iparm[9] = 13;        /* Perturb the pivot elements with 1E-13 */
	iparm[10] = 1;        /* Use nonsymmetric permutation and scaling MPS */
	iparm[11] = 0;        /* Not in use */
	iparm[12] = 0;        /* Maximum weighted matching algorithm is switched-off (default for symmetric). Try iparm[12] = 1 in case of inappropriate accuracy */
	iparm[13] = 0;        /* Output: Number of perturbed pivots */
	iparm[14] = 0;        /* Not in use */
	iparm[15] = 0;        /* Not in use */
	iparm[16] = 0;        /* Not in use */
	iparm[17] = -1;       /* Output: Number of nonzeros in the factor LU */
	iparm[18] = -1;       /* Output: Mflops for LU factorization */
	iparm[19] = 0;        /* Output: Numbers of CG Iterations */

}

void mkl_eq_solver::choleskyfactorize()
{
	phase = 12; /* .. Reordering and Symbolic Factorization. This step also allocates all memory that is necessary for the factorization. */
	PARDISO(pt, &maxfct, &mnum, &mtype, &phase, &n, a, ia, ja, &idum, &nrhs, iparm, &msglvl, &ddum, &ddum, &error);
	if (error != 0)
	{
		printf("\nERROR during reordering or symbolic factorization: %d", error);
		exit(1);
	}

}

void mkl_eq_solver::iterativeSolve(double *bfield)
{
	b = bfield;
	phase = 33; /* .. Back substitution and iterative refinement. */
	PARDISO(pt, &maxfct, &mnum, &mtype, &phase, &n, a, ia, ja, &idum, &nrhs, iparm, &msglvl, b, x, &error);
	if (error != 0)
	{
		printf("\nERROR during solution: %d", error);
		exit(3);
	}
}



