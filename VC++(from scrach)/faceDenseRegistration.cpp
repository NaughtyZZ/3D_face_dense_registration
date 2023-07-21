// faceDenseRegistration.cpp£ºmain project File
//
#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <io.h>
#include "./Eigen/SparseCore"
#include "meshdata.h"
#include "mkl_solve_eq.h"
#include <time.h>
#include <Eigen/SVD>


int main()
{

	clock_t clockBegin, clockEnd;
	PointCloud templateFace, targetFace;


	templateFace.loadPlyFile("template.ply"); // load template
	templateFace.computeVertexAndCellRing1(); //compute 1-rings neighboring relations
	templateFace.loadLandmarksIdx("idxF.txt"); //load landmarks indices
	templateFace.loadWeightsFromDfield("weights.txt"); //load pre-computed weights according to geodesic distances	


	targetFace.loadPlyFile("target.ply"); // load target
	targetFace.buildAABBTree(); // build AABB tree for target
	targetFace.loadLandmarkPositions("target.pp"); // load landmark positions for the target
	targetFace.computeOuterEdgeIdx(); // compute boundary of the mesh



	int i, j;
	SpMat A_Matrix(templateFace.N_v, templateFace.N_v);
	SpMat B_Matrix(templateFace.N_v, templateFace.N_v);

	for (i = 0; i < templateFace.N_v; i++)
	{
		A_Matrix.insert(i, i) = 1; //identity matrix
	}

	for (i = 0; i < templateFace.N_v; i++)
	{
		B_Matrix.insert(i, i) = templateFace.numberRing1[i];
		for (j = 0; j < templateFace.numberRing1[i]; j++)
		{
			B_Matrix.insert(i, templateFace.vertexRing1Idx[i][j]) = -1;
		}

	}
	SpMat C_Matrix = A_Matrix + B_Matrix;
	SpMat A_M_Matrix(templateFace.N_v - templateFace.N_k, templateFace.N_v);

	//sorting landmark indices and exclude the corresponding columns 
	std::vector<int> myVec(templateFace.landmarks, templateFace.landmarks + templateFace.N_k);
	sort(myVec.begin(), myVec.end());
	for (i = 0, j = 0; i < C_Matrix.outerSize(); ++i)
	{

		if (j < (templateFace.N_k) && i == myVec[j])
		{
			j++;
			continue;
		}
		else
		{
			for (Eigen::SparseMatrix<double, RowMajor>::InnerIterator it(C_Matrix, i); it; ++it)
			{
				A_M_Matrix.insert(i - j, it.col()) = it.value();
			}
			//system("pause");
		}

	}

	//initilize symmetric matrix and solvers
	SpMat A_SYMM = A_M_Matrix*A_M_Matrix.transpose();
	mkl_eq_solver sparseSolver(A_SYMM);
	sparseSolver.choleskyfactorize();


	clockBegin = clock();
	templateFace.initialDeformation(targetFace);
	templateFace.writePlyFile("initialTemplate.ply");
	templateFace.denseRegistration(targetFace, sparseSolver, A_M_Matrix, 100);
	templateFace.writePlyFile("output.ply");
	clockEnd = clock();

	cout << "Time elapsed: " << (double)(clockEnd - clockBegin) / CLOCKS_PER_SEC << "s (CPU)" << endl;
	system("pause");
	return 0;
}