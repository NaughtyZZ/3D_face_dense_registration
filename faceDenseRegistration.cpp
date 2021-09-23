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


//list file in a folder
void getFiles(string path, string exd, vector<string>& files)
{
	//file handle
	intptr_t hFile = 0;
	//file information 
	struct _finddata_t fileinfo; 
	string pathName, exdName;
	if (0 != strcmp(exd.c_str(), "")) {
		exdName = "*." + exd;
	}
	else {
		exdName = "*";
	}
	if ((hFile = _findfirst(pathName.assign(path).append(exdName).c_str(), &fileinfo)) != -1)
	{
		do
		{
			//add file name to List 
			{
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0) {
					files.push_back(fileinfo.name); 
				}
			}
		} 
		while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}

int main(int argc, char** argv)
{

	clock_t clockBegin, clockEnd, totalClockBegin, totalClockEnd;
	PointCloud templateFace, targetFace;

	// load template and its MRA indices
	templateFace.loadPlyFile("template.ply");
	templateFace.computeVertexAndCellRing1();
	templateFace.MRA_index.loadIdxCatogory(".\\MRA_txt\\idxK.txt", ".\\MRA_txt\\idxE.txt", ".\\MRA_txt\\idxF.txt", ".\\MRA_txt\\idxD.txt");
	templateFace.MRA_index.loadMRA_0(".\\MRA_txt\\M0_all.txt", ".\\MRA_txt\\M0_fixed.txt", ".\\MRA_txt\\M0_edges.txt");
	templateFace.MRA_index.loadMRA_1(".\\MRA_txt\\M1_all.txt", ".\\MRA_txt\\M1_fixed.txt", ".\\MRA_txt\\M1_edges.txt");
	templateFace.MRA_index.loadMRA_2(".\\MRA_txt\\M2_all.txt", ".\\MRA_txt\\M2_fixed.txt", ".\\MRA_txt\\M2_edges.txt");
	templateFace.MRA_index.loadFreeIndices(".\\MRA_txt\\M0_free.txt", ".\\MRA_txt\\M1_free.txt", ".\\MRA_txt\\M2_free.txt", ".\\MRA_txt\\M3_free.txt", ".\\MRA_txt\\M_free.txt");
	templateFace.computeMRAindex();
	templateFace.loadVertexWeights(".\\MRA_txt\\weight_v.txt");

	double lamda = 20;//local smoothing parameter
	int i, j;

	// initilize matrices for the equation (A+lamda*B)*O=D  Resolution 0 
	SpMat A_Matrix0(templateFace.MRA_index.N0, templateFace.MRA_index.N0);
	SpMat B_Matrix0(templateFace.MRA_index.N0, templateFace.MRA_index.N0);

	for (i = 0; i < templateFace.MRA_index.N0; i++)
	{
		A_Matrix0.insert(i, i) = 1;
		B_Matrix0.insert(i, i) = templateFace.MRA_N0[i];
		for (j = 0; j < templateFace.MRA_N0[i]; j++)
		{
			B_Matrix0.insert(i, templateFace.MRA_vertexIdx0[i][j]) = -1;
		}
	}

	SpMat C_Matrix0 = A_Matrix0 + lamda*B_Matrix0;

	//Compute rank deficient matrix A_M(size N*(N-M))  
	SpMat A_M_Matrix0(templateFace.MRA_index.N0 - templateFace.MRA_index.F0, templateFace.MRA_index.N0);
	std::vector<int> myVec0(templateFace.MRA_index.fIdx0, templateFace.MRA_index.fIdx0 + templateFace.MRA_index.F0);
	sort(myVec0.begin(), myVec0.end());
	for (i = 0, j = 0; i < C_Matrix0.outerSize(); ++i)
	{

		if (j < templateFace.MRA_index.F0&&myVec0[j] == i)
		{
			j++;
			continue;
		}
		else
		{
			for (Eigen::SparseMatrix<double, RowMajor>::InnerIterator it(C_Matrix0, i); it; ++it)
			{
				A_M_Matrix0.insert(i - j, it.col()) = it.value();
			}

		}

	}
	SpMat A_SYMM0 = A_M_Matrix0*A_M_Matrix0.transpose();
	//solve sparse equation efficiently using intel's MKL library
	mkl_eq_solver sparseSolver0(A_SYMM0);
	sparseSolver0.choleskyfactorize();

	// Resolution 1
	SpMat A_Matrix1(templateFace.MRA_index.N1, templateFace.MRA_index.N1);
	SpMat B_Matrix1(templateFace.MRA_index.N1, templateFace.MRA_index.N1);

	for (i = 0; i < templateFace.MRA_index.N1; i++)
	{
		A_Matrix1.insert(i, i) = 1;
		B_Matrix1.insert(i, i) = templateFace.MRA_N1[i];
		for (j = 0; j < templateFace.MRA_N1[i]; j++)
		{
			B_Matrix1.insert(i, templateFace.MRA_vertexIdx1[i][j]) = -1;
		}
	}

	SpMat C_Matrix1 = A_Matrix1 + lamda*B_Matrix1;
	SpMat A_M_Matrix1(templateFace.MRA_index.N1 - templateFace.MRA_index.F1, templateFace.MRA_index.N1);

	std::vector<int> myVec1(templateFace.MRA_index.fIdx1, templateFace.MRA_index.fIdx1 + templateFace.MRA_index.F1);
	sort(myVec1.begin(), myVec1.end());
	for (i = 0, j = 0; i < C_Matrix1.outerSize(); ++i)
	{

		if (j < templateFace.MRA_index.F1&&myVec1[j] == i)
		{
			j++;
			continue;
		}
		else
		{
			for (Eigen::SparseMatrix<double, RowMajor>::InnerIterator it(C_Matrix1, i); it; ++it)
			{
				A_M_Matrix1.insert(i - j, it.col()) = it.value();
			}

		}

	}
	SpMat A_SYMM1 = A_M_Matrix1*A_M_Matrix1.transpose();
	mkl_eq_solver sparseSolver1(A_SYMM1);
	sparseSolver1.choleskyfactorize();



	// Resolution 2
	SpMat A_Matrix2(templateFace.MRA_index.N2, templateFace.MRA_index.N2);
	SpMat B_Matrix2(templateFace.MRA_index.N2, templateFace.MRA_index.N2);

	for (i = 0; i < templateFace.MRA_index.N2; i++)
	{
		A_Matrix2.insert(i, i) = 1;
		B_Matrix2.insert(i, i) = templateFace.MRA_N2[i];
		for (j = 0; j < templateFace.MRA_N2[i]; j++)
		{
			B_Matrix2.insert(i, templateFace.MRA_vertexIdx2[i][j]) = -1;
		}
	}

	SpMat C_Matrix2 = A_Matrix2 + lamda*B_Matrix2;
	SpMat A_M_Matrix2(templateFace.MRA_index.N2 - templateFace.MRA_index.F2, templateFace.MRA_index.N2);

	std::vector<int> myVec2(templateFace.MRA_index.fIdx2, templateFace.MRA_index.fIdx2 + templateFace.MRA_index.F2);
	sort(myVec2.begin(), myVec2.end());
	for (i = 0, j = 0; i < C_Matrix2.outerSize(); ++i)
	{

		if (j < templateFace.MRA_index.F2&&myVec2[j] == i)
		{
			j++;
			continue;
		}
		else
		{
			for (Eigen::SparseMatrix<double, RowMajor>::InnerIterator it(C_Matrix2, i); it; ++it)
			{
				A_M_Matrix2.insert(i - j, it.col()) = it.value();
			}

		}

	}
	SpMat A_SYMM2 = A_M_Matrix2*A_M_Matrix2.transpose();
	mkl_eq_solver sparseSolver2(A_SYMM2);
	sparseSolver2.choleskyfactorize();

	// Resolution 3 and Full Resolution
	SpMat A_Matrix(templateFace.N_v, templateFace.N_v);
	SpMat B_Matrix(templateFace.N_v, templateFace.N_v);

	for (i = 0; i < templateFace.N_v; i++)
	{
		A_Matrix.insert(i, i) = 1;
	}

	for (i = 0; i < templateFace.N_v; i++)
	{
		B_Matrix.insert(i, i) = templateFace.numberRing1[i]*templateFace.vertexWeights[i];

		//double tmpValue=0;
		//for (j = 0; j < templateFace.numberRing1[i]; j++)
		//{
		//	double tmpValue2 = (templateFace.points.row(i) - templateFace.points.row(templateFace.vertexRing1Idx[i][j])).norm();
		//	tmpValue2 = -19.8*tmpValue*tmpValue / 140 + 20;
		//	if (j == 0)
		//		tmpValue = tmpValue2;
		//	else if (tmpValue > tmpValue2)
		//		tmpValue = tmpValue2;


		//	
		//	//B_Matrix.insert(i, templateFace.vertexRing1Idx[i][j]) = -tmpValue;
		//	//sum_ring += tmpValue;
		//}
		//if (tmpValue > 20)
		//	tmpValue = 20;
		//else if (tmpValue <0)
		//	tmpValue = 0;
		//B_Matrix.insert(i, i) = tmpValue*templateFace.numberRing1[i];

		for (j = 0; j < templateFace.numberRing1[i]; j++)
		{			
			B_Matrix.insert(i, templateFace.vertexRing1Idx[i][j]) = -templateFace.vertexWeights[i];
		}

		//double sum_ring = 0;
		//for (j = 0; j < templateFace.numberRing1[i]; j++)
		//{			
		//	double tmpValue = (templateFace.points.row(i) - templateFace.points.row(templateFace.vertexRing1Idx[i][j])).norm();
		//	//tmpValue = 10 / tmpValue;
		//	tmpValue = -20*tmpValue + 40;
		//	if (tmpValue > 20)
		//		tmpValue = 20;
		//	else if(tmpValue <1)
		//		tmpValue = 1;
		//	B_Matrix.insert(i, templateFace.vertexRing1Idx[i][j]) = -tmpValue;
		//	sum_ring += tmpValue;
		//}
		//B_Matrix.insert(i, i) = sum_ring;

	}
	
	//SpMat C_Matrix = A_Matrix + lamda*B_Matrix;
	SpMat C_Matrix = A_Matrix + B_Matrix;


	// added subsequently for landmarks bendings
	//for (i = 0; i < templateFace.MRA_index.N_idxK; i++)
	//{
	//	C_Matrix.row(templateFace.MRA_index.idxK[i]) *= 100;
	//}
	//for (i = 0; i < templateFace.MRA_index.N_idxE; i++)
	//{
	//	C_Matrix.row(templateFace.MRA_index.idxE[i]) *= 100;
	//}



	//Full Resolution: copy and sort Fixed point indices
	SpMat A_M_Matrix(templateFace.MRA_index.N_idxD, templateFace.N_v);
	int *fixedPointIdx = new int[templateFace.N_v - templateFace.MRA_index.N_idxD];
	memcpy(fixedPointIdx, templateFace.MRA_index.idxK, templateFace.MRA_index.N_idxK * sizeof(int));
	memcpy(fixedPointIdx + templateFace.MRA_index.N_idxK, templateFace.MRA_index.idxE, templateFace.MRA_index.N_idxE * sizeof(int));
	memcpy(fixedPointIdx + templateFace.MRA_index.N_idxK + templateFace.MRA_index.N_idxE, templateFace.MRA_index.idxF, templateFace.MRA_index.N_idxF * sizeof(int));
	std::vector<int> myVec(fixedPointIdx, fixedPointIdx + templateFace.N_v - templateFace.MRA_index.N_idxD);
	sort(myVec.begin(), myVec.end());
	for (i = 0, j = 0; i < C_Matrix.outerSize(); ++i)
	{

		if (j < (templateFace.N_v - templateFace.MRA_index.N_idxD) && myVec[j] == i)
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

	//Resolution 3: copy and sort Fixed point indices
	SpMat A_M_Matrix3(templateFace.N_v- templateFace.MRA_index.N_idxF- templateFace.MRA_index.N2, templateFace.N_v);
	int *fIdx = new int[templateFace.MRA_index.N_idxF + templateFace.MRA_index.N2];
	memcpy(fIdx, templateFace.MRA_index.pIdx2, templateFace.MRA_index.N2 * sizeof(int));
	memcpy(fIdx + templateFace.MRA_index.N2, templateFace.MRA_index.idxF, templateFace.MRA_index.N_idxF * sizeof(int));
	std::vector<int> myVec3(fIdx, fIdx + templateFace.MRA_index.N_idxF + templateFace.MRA_index.N2);
	sort(myVec3.begin(), myVec3.end());
	for (i = 0, j = 0; i < C_Matrix.outerSize(); ++i)
	{

		if (j < (templateFace.MRA_index.N_idxF + templateFace.MRA_index.N2) && myVec3[j] == i)
		{
			j++;
			continue;
		}
		else
		{
			for (Eigen::SparseMatrix<double, RowMajor>::InnerIterator it(C_Matrix, i); it; ++it)
			{
				A_M_Matrix3.insert(i - j, it.col()) = it.value();
			}

		}

	}
	//Full Resolution: initilize symmetric matrix and solvers
	SpMat A_SYMM = A_M_Matrix*A_M_Matrix.transpose();
	mkl_eq_solver sparseSolver(A_SYMM);
	sparseSolver.choleskyfactorize();

	//Resolution 3: initilize symmetric matrix and solvers
	SpMat A_SYMM3 = A_M_Matrix3*A_M_Matrix3.transpose();
	mkl_eq_solver sparseSolver3(A_SYMM3);
	sparseSolver3.choleskyfactorize();


	totalClockBegin = clock();
	vector<string> files;
	getFiles(argv[1], "ply", files);
	for (int ii = 0; ii < files.size(); ii++)
	{
		//clockBegin= clock();
		string currentPlyFile = argv[1] + files[ii];
		//string currentTxtFile = argv[2] + files[ii].substr(0, currentPlyFile.size() - 4) + ".txt";
		string outputPlyFile = argv[2] + files[ii];
		targetFace.loadPlyFile(currentPlyFile);
		targetFace.buildAABBTree();
		//targetFace.rectifyLandmarks(".\\MRA_txt\\idxK.txt", currentTxtFile);
		Eigen::MatrixXd OriginalTarget = targetFace.points;
		//cout << "Time elapsed: " << (double)(clock() - clockBegin) / CLOCKS_PER_SEC << "s (CPU)" << endl;
		//clockBegin = clock();
		//Multiresolution iterative mesh correction
		//targetFace.rectifyMRAVertices(templateFace, sparseSolver0, OriginalTarget, A_M_Matrix0, 1000, Resolution_0);
		//targetFace.writePlyFile(outputPlyFile);
		//system("pause");
		//cout << "Time elapsed: " << (double)(clock() - clockBegin) / CLOCKS_PER_SEC << "s (CPU)" << endl;
		//clockBegin = clock();
		//targetFace.rectifyMRAVertices(templateFace, sparseSolver1, OriginalTarget, A_M_Matrix1, 1000, Resolution_1);
		//targetFace.writePlyFile(outputPlyFile);
		//system("pause");
		//cout << "Time elapsed: " << (double)(clock() - clockBegin) / CLOCKS_PER_SEC << "s (CPU)" << endl;
		//clockBegin = clock();
		//targetFace.rectifyMRAVertices(templateFace, sparseSolver2, OriginalTarget, A_M_Matrix2, 1000, Resolution_2);
		//targetFace.writePlyFile(outputPlyFile);
		//system("pause");
		//cout << "Time elapsed: " << (double)(clock() - clockBegin) / CLOCKS_PER_SEC << "s (CPU)" << endl;
		//clockBegin = clock();
		//targetFace.rectifyMRAVertices(templateFace, sparseSolver3, OriginalTarget, A_M_Matrix3, 1000, Resolution_3);
		//targetFace.writePlyFile(outputPlyFile);
		//system("pause");
		//cout << "Time elapsed: " << (double)(clock() - clockBegin) / CLOCKS_PER_SEC << "s (CPU)" << endl;
		//clockBegin = clock();
		targetFace.rectifyMRAVertices(templateFace, sparseSolver, OriginalTarget, A_M_Matrix, 1000, Full_Resolution);
		//targetFace.writePlyFile(outputPlyFile);
		//system("pause");
		//cout << "Time elapsed: " << (double)(clock() - clockBegin) / CLOCKS_PER_SEC << "s (CPU)" << endl;
		//clockBegin = clock();
		targetFace.writePlyFile(outputPlyFile);
		//system("pause");
	}

	totalClockEnd = clock();
	cout << "Time elapsed: " << (double)(totalClockEnd - totalClockBegin) / CLOCKS_PER_SEC << "s (CPU)" << endl;
	system("pause");
	return 0;
}