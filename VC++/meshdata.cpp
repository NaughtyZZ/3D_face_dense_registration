#pragma once
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <igl/per_vertex_normals.h>
//#include "AABB.h"
//#include <regex>
#include "meshdata.h"
//using namespace std;
//load point indices and its number
int loadIdxFile(string FName, int **idxList)
{
	ifstream ifile;
	string s;
	int count = 0;

	ifile.open(FName.c_str(), ifstream::in);
	if (!ifile.is_open())
	{
		cout << "Unable to open point file '" << FName << "'" << endl;
		exit(-1);
	}
	getline(ifile, s);
	istringstream ss1(s);
	string sTmp;
	while (ss1 >> sTmp)
		count++;
	istringstream ss2(s);
	*idxList = new int[count];
	count = 0;
	while (ss2 >> sTmp)
		(*idxList)[count++] = atoi(sTmp.c_str());
	ifile.close();
	return count;

}


void loadWeightFile(string FName, double **weight_v)
{
	ifstream ifile;
	string s;
	int count = 0;

	ifile.open(FName.c_str(), ifstream::in);
	if (!ifile.is_open())
	{
		cout << "Unable to open point file '" << FName << "'" << endl;
		exit(-1);
	}
	getline(ifile, s);
	istringstream ss1(s);
	string sTmp;
	while (ss1 >> sTmp)
		count++;
	istringstream ss2(s);
	*weight_v = new double[count];
	count = 0;
	while (ss2 >> sTmp)
		(*weight_v)[count++] = atof(sTmp.c_str());
	ifile.close();


}




//template <typename DerivedV, 3>
// initilize
PointCloud::PointCloud()
{
	N_v = 0;
	N_f = 0;
	//N_k = 0;
	//N_l = 0;
	points(N_v, 3);
	normals(N_v, 3);
	textures(N_v, 3);
	triList(N_f, 3);
	vertexRing1Idx = NULL;
	numberRing1 = NULL;
	cell_ring1 = NULL;

}

//template <typename DerivedV, 3>
//load pointCloud from *ply file
void PointCloud::loadPlyFile(string FName)
{
	PointCloud();
	int i;
	//N1 = 0;
	//N2 = 0;
	ifstream ifile;
	string s1, s2, s3, s4, s5, s6, s7, s8, s9, s10, s11;
	ifile.open(FName.c_str(), ifstream::in);
	if (!ifile.is_open())
	{
		cout << "Unable to open point file '" << FName << "'" << endl;
		exit(-1);
	}

	while (getline(ifile, s1))
	{
		s11 = s1.substr(0, 3);
		if (s11 == "ele")
		{
			if (N_v == 0)
			{
				istringstream ss1(s1);
				ss1 >> s2 >> s3 >> s4;
				N_v = atoi(s4.c_str());
			}
			else
			{
				istringstream ss1(s1);
				ss1 >> s2 >> s3 >> s4;
				N_f = atoi(s4.c_str());
			}
		}
		else if (s11 == "end")
			break;

	}

	points = Eigen::MatrixXd::Zero(N_v, 3);
	normals = Eigen::MatrixXd::Zero(N_v, 3);
	textures = Eigen::MatrixXi::Zero(N_v, 3);
	triList = Eigen::MatrixXi::Zero(N_f, 3);


	for (i = 0; i < N_v; i++)
	{
		getline(ifile, s1);
		istringstream ss1(s1);
		ss1 >> s2 >> s3 >> s4 >> s5 >> s6 >> s7 >> s11 >> s8 >> s9 >> s10 >> s11;
		points(i, 0) = atof(s2.c_str());
		points(i, 1) = atof(s3.c_str());
		points(i, 2) = atof(s4.c_str());

		normals(i, 0) = atof(s5.c_str());
		normals(i, 1) = atof(s6.c_str());
		normals(i, 2) = atof(s7.c_str());

		textures(i, 0) = atoi(s8.c_str());
		textures(i, 1) = atoi(s9.c_str());
		textures(i, 2) = atoi(s10.c_str());
		//ifile >> (points)[i].x >> (points)[i].y >> (points)[i].z >> (normals)[i].x >> (normals)[i].y >> (normals)[i].z >> N_null >> (textures)[i].y >> (textures)[i].z >> (textures)[i].x >> N_null;
	}

	for (i = 0; i < N_f; i++)
	{
		getline(ifile, s1);
		istringstream ss1(s1);
		ss1 >> s11 >> s2 >> s3 >> s4 >> s11;
		triList(i, 0) = atoi(s2.c_str());
		triList(i, 1) = atoi(s3.c_str());
		triList(i, 2) = atoi(s4.c_str());
		//ifile >> N_null >> (triList)[i].x >> (triList)[i].y >> (triList)[i].z >> N_null;
	}

	ifile.close();
}
//write pointCloud to *.ply file
void PointCloud::writePlyFile(string FName)
{
	int i;
	ofstream ofile;
	ofile.open(FName.c_str(), ofstream::out);
	ofile << "ply" << endl;
	ofile << "format ascii 1.0" << endl;
	ofile << "element vertex" << " " << N_v << endl;
	ofile << "property float x" << endl;
	ofile << "property float y" << endl;
	ofile << "property float z" << endl;
	ofile << "property float nx" << endl;
	ofile << "property float ny" << endl;
	ofile << "property float nz" << endl;
	ofile << "property int flags" << endl;
	ofile << "property uchar red" << endl;
	ofile << "property uchar green" << endl;
	ofile << "property uchar blue" << endl;
	ofile << "property uchar alpha" << endl;
	ofile << "element face" << " " << N_f << endl;
	ofile << "property list uchar int vertex_indices" << endl;
	ofile << "property int flags" << endl;
	ofile << "end_header" << endl;

	for (i = 0; i < N_v; i++)
	{

		ofile << points(i, 0) << " " << points(i, 1) << " " << points(i, 2) << " " << normals(i, 0) << " " << normals(i, 1) << " " << normals(i, 2) << " " << 0 << " " << textures(i, 0) << " " << textures(i, 1) << " " << textures(i, 2) << " " << 255 << endl;
	}

	for (i = 0; i < N_f; i++)
	{
		ofile << 3 << " " << triList(i, 0) << " " << triList(i, 1) << " " << triList(i, 2) << " " << 0 << endl;
	}

	ofile.close();
}

//template <typename DerivedV, 3>
//update normals
void PointCloud::updateNormals()
{
	//Eigen::MatrixXd normalsTmp= normals;
	igl::per_vertex_normals(points, triList, normals);

	normals = (normals.array().isFinite()).select(normals, 0.0);


	//normals= normalsTmp + (normals- normalsTmp).array()*normals.array().isFinite();

	//Eigen::MatrixXd e(2, 2);
	//e << 8.0, std::nan("1"), std::numeric_limits <double> ::infinity(), 4.0;
	//std::cout << e << std::endl;

	//std::cout << e.array()*e.array().isFinite()<< std::endl;

	////Eigen::MatrixXd cc= e.array().isFinite().matrix();
	//e = (e.array().isFinite()).select(e, 0.0);
	//std::cout << e << std::endl;

	//system("pause");

	//cout << normals.array().isNaN() << endl;
	//system("pause");
}

//template <typename DerivedV, 3>
//build AABB tree
void PointCloud::buildAABBTree()
{
	tree.init(points, triList);
}
//compute adjacenty
void PointCloud::computeVertexAndCellRing1()
{
	numberRing1 = new int[N_v];
	vertexRing1Idx = new int*[N_v];
	int i, j, count, count2;
	int *temArray = new int[100];
	for (i = 0; i < N_v; i++)
	{
		count = 0;
		for (j = 0; j < N_f; j++)
		{
			if (triList(j, 0) == i || triList(j, 1) == i || triList(j, 2) == i)
			{
				temArray[count++] = triList(j, 0);
				temArray[count++] = triList(j, 1);
				temArray[count++] = triList(j, 2);
			}

		}
		//unique();
		std::vector<int> myVec(temArray, temArray + count);
		sort(myVec.begin(), myVec.end());
		myVec.erase(unique(myVec.begin(), myVec.end()), myVec.end());
		//renew temArray[]
		count = myVec.size();
		numberRing1[i] = count - 1;
		vertexRing1Idx[i] = new int[count - 1];
		memcpy(temArray, &myVec[0], myVec.size() * sizeof(int));
		myVec.clear();
		//Store the values
		count2 = 0;
		for (j = 0; j < count; j++)
		{
			if (temArray[j] != i)
				vertexRing1Idx[i][count2++] = temArray[j];
		}
	}
	cell_ring1 = new Eigen::MatrixXd[N_v];
	for (i = 0; i < N_v; i++)
	{
		cell_ring1[i] = Eigen::MatrixXd::Zero(numberRing1[i], 3);
		for (j = 0; j < numberRing1[i]; j++)
		{
			cell_ring1[i].row(j) = points.row(vertexRing1Idx[i][j]) - points.row(i);
			//cell_ring1[i](j, 0) = points(vertexRing1Idx[i][j], 0) - points(i, 0);
			//cell_ring1[i](j, 1) = points(vertexRing1Idx[i][j], 1) - points(i, 1);
			//cell_ring1[i](j, 2) = points(vertexRing1Idx[i][j], 2) - points(i, 2);
		}
	}
}
// compute MRA adjacenty
void PointCloud::computeMRAindex()
{
	int i, j;

	MRA_N0 = new int[MRA_index.N0];
	memset(MRA_N0, 0, MRA_index.N0 * sizeof(int));
	for (i = 0; i < MRA_index.M0; i++)
	{
		MRA_N0[MRA_index.mIdx0[2 * i]] += 1;
		MRA_N0[MRA_index.mIdx0[2 * i + 1]] += 1;
	}

	MRA_vertexIdx0 = new int*[MRA_index.N0];
	for (i = 0; i < MRA_index.N0; i++)
	{
		MRA_vertexIdx0[i] = new int[MRA_N0[i]];
		memset(MRA_vertexIdx0[i], -1, MRA_N0[i] * sizeof(int));
	}

	int *count0 = new int[MRA_index.N0];
	memset(count0, 0, MRA_index.N0 * sizeof(int));
	for (i = 0; i < MRA_index.M0; i++)
	{
		MRA_vertexIdx0[MRA_index.mIdx0[2 * i]][count0[MRA_index.mIdx0[2 * i]]++] = MRA_index.mIdx0[2 * i + 1];
		MRA_vertexIdx0[MRA_index.mIdx0[2 * i + 1]][count0[MRA_index.mIdx0[2 * i + 1]]++] = MRA_index.mIdx0[2 * i];
		//if (MRA_index.mIdx0[2 * i] == 110)
		//{
		//	cout << MRA_index.mIdx0[2 * i] << endl;
		//	cout << MRA_index.mIdx0[2 * i+1] << endl;
		//	system("pause");
		//}
	}

	MRA_cell0 = new Eigen::MatrixXd[MRA_index.N0];
	for (i = 0; i < MRA_index.N0; i++)
	{
		MRA_cell0[i] = Eigen::MatrixXd::Zero(MRA_N0[i], 3);
		for (j = 0; j < MRA_N0[i]; j++)
		{
			//cout <<i<<"  "<<j<<"  "<< MRA_vertexIdx0[i][j] <<"  "<< MRA_index.pIdx0[MRA_vertexIdx0[i][j]] <<" "<< MRA_index.pIdx0[i]<<endl;
			MRA_cell0[i].row(j) = points.row(MRA_index.pIdx0[MRA_vertexIdx0[i][j]]) - points.row(MRA_index.pIdx0[i]);
		}
	}


	MRA_N1 = new int[MRA_index.N1];
	memset(MRA_N1, 0, MRA_index.N1 * sizeof(int));
	for (i = 0; i < MRA_index.M1; i++)
	{
		MRA_N1[MRA_index.mIdx1[2 * i]] += 1;
		MRA_N1[MRA_index.mIdx1[2 * i + 1]] += 1;
	}

	MRA_vertexIdx1 = new int*[MRA_index.N1];
	for (i = 0; i < MRA_index.N1; i++)
	{
		MRA_vertexIdx1[i] = new int[MRA_N1[i]];
		memset(MRA_vertexIdx1[i], -1, MRA_N1[i] * sizeof(int));
	}

	int *count1 = new int[MRA_index.N1];
	memset(count1, 0, MRA_index.N1 * sizeof(int));
	for (i = 0; i < MRA_index.M1; i++)
	{
		//cout << MRA_index.mIdx1[2 * i + 1] <<"  "<< count0[MRA_index.mIdx1[2 * i + 1]] << endl;
		MRA_vertexIdx1[MRA_index.mIdx1[2 * i]][count1[MRA_index.mIdx1[2 * i]]++] = MRA_index.mIdx1[2 * i + 1];
		MRA_vertexIdx1[MRA_index.mIdx1[2 * i + 1]][count1[MRA_index.mIdx1[2 * i + 1]]++] = MRA_index.mIdx1[2 * i];
	}

	MRA_cell1 = new Eigen::MatrixXd[MRA_index.N1];
	for (i = 0; i < MRA_index.N1; i++)
	{
		MRA_cell1[i] = Eigen::MatrixXd::Zero(MRA_N1[i], 3);
		for (j = 0; j < MRA_N1[i]; j++)
		{
			MRA_cell1[i].row(j) = points.row(MRA_index.pIdx1[MRA_vertexIdx1[i][j]]) - points.row(MRA_index.pIdx1[i]);
		}
	}


	MRA_N2 = new int[MRA_index.N2];
	memset(MRA_N2, 0, MRA_index.N2 * sizeof(int));
	for (i = 0; i < MRA_index.M2; i++)
	{
		MRA_N2[MRA_index.mIdx2[2 * i]] += 1;
		MRA_N2[MRA_index.mIdx2[2 * i + 1]] += 1;
	}

	MRA_vertexIdx2 = new int*[MRA_index.N2];
	for (i = 0; i < MRA_index.N2; i++)
	{
		MRA_vertexIdx2[i] = new int[MRA_N2[i]];
		memset(MRA_vertexIdx2[i], -1, MRA_N2[i] * sizeof(int));
	}

	int *count2 = new int[MRA_index.N2];
	memset(count2, 0, MRA_index.N2 * sizeof(int));
	for (i = 0; i < MRA_index.M2; i++)
	{
		MRA_vertexIdx2[MRA_index.mIdx2[2 * i]][count2[MRA_index.mIdx2[2 * i]]++] = MRA_index.mIdx2[2 * i + 1];
		MRA_vertexIdx2[MRA_index.mIdx2[2 * i + 1]][count2[MRA_index.mIdx2[2 * i + 1]]++] = MRA_index.mIdx2[2 * i];
	}

	MRA_cell2 = new Eigen::MatrixXd[MRA_index.N2];
	for (i = 0; i < MRA_index.N2; i++)
	{
		MRA_cell2[i] = Eigen::MatrixXd::Zero(MRA_N2[i], 3);
		for (j = 0; j < MRA_N2[i]; j++)
		{
			MRA_cell2[i].row(j) = points.row(MRA_index.pIdx2[MRA_vertexIdx2[i][j]]) - points.row(MRA_index.pIdx2[i]);
		}
	}

}
//MRA registration and update
void PointCloud::rectifyMRAVertices(PointCloud templateFace, mkl_eq_solver spSolver, Eigen::MatrixXd OriginalTarget, SpMat A_Re_Matrix, int maxIter, MRA flag)
{
	int i, j, jj;
	Eigen::MatrixXd Dfield = Eigen::MatrixXd::Zero(templateFace.N_v, 3);
	Eigen::MatrixXd Dfield_T;
	Eigen::MatrixXd Dfield_M;

	int N, N_FV;//number of all vertices and fixed vertices
	Eigen::MatrixXd *MRA_cell = NULL;//neighboring cell
	int *localAdjNum = NULL;//number of vertices for 1-ring neighbor of each vertex
	int *pIdx = NULL;//indices referring to the whole face
	int **MRA_vertexIdx = NULL;//indices for 1-ring neighbors
	int *fIdx = NULL;//indices for fixed vertices

	int N_free;//number of free vertices within interested mask;
	int *wIdx;//Indices of free vertices within interested mask;

	switch (flag)
	{
	case Resolution_0:
		N = templateFace.MRA_index.N0;
		N_FV = templateFace.MRA_index.F0;
		MRA_cell = templateFace.MRA_cell0;
		localAdjNum = templateFace.MRA_N0;
		pIdx = templateFace.MRA_index.pIdx0;
		MRA_vertexIdx = templateFace.MRA_vertexIdx0;
		fIdx = templateFace.MRA_index.fIdx0;
		N_free = templateFace.MRA_index.N_R0;
		wIdx = templateFace.MRA_index.wIdx0;
		break;
	case Resolution_1:
		N = templateFace.MRA_index.N1;
		N_FV = templateFace.MRA_index.F1;
		MRA_cell = templateFace.MRA_cell1;
		localAdjNum = templateFace.MRA_N1;
		pIdx = templateFace.MRA_index.pIdx1;
		MRA_vertexIdx = templateFace.MRA_vertexIdx1;
		fIdx = templateFace.MRA_index.fIdx1;
		N_free = templateFace.MRA_index.N_R1;
		wIdx = templateFace.MRA_index.wIdx1;
		break;
	case Resolution_2:
		N = templateFace.MRA_index.N2;
		N_FV = templateFace.MRA_index.F2;
		MRA_cell = templateFace.MRA_cell2;
		localAdjNum = templateFace.MRA_N2;
		pIdx = templateFace.MRA_index.pIdx2;
		MRA_vertexIdx = templateFace.MRA_vertexIdx2;
		fIdx = templateFace.MRA_index.fIdx2;
		N_free = templateFace.MRA_index.N_R2;
		wIdx = templateFace.MRA_index.wIdx2;
		break;
	case Resolution_3:
	{
		N = templateFace.N_v;
		N_FV = templateFace.MRA_index.N_idxF + templateFace.MRA_index.N2;
		MRA_cell = templateFace.cell_ring1;
		localAdjNum = templateFace.numberRing1;

		fIdx = new int[templateFace.MRA_index.N_idxF + templateFace.MRA_index.N2];
		memcpy(fIdx, templateFace.MRA_index.pIdx2, templateFace.MRA_index.N2 * sizeof(int));
		memcpy(fIdx + templateFace.MRA_index.N2, templateFace.MRA_index.idxF, templateFace.MRA_index.N_idxF * sizeof(int));
		std::vector<int> myVec(fIdx, fIdx + templateFace.MRA_index.N_idxF + templateFace.MRA_index.N2);
		sort(myVec.begin(), myVec.end());
		memcpy(fIdx, &myVec[0], myVec.size() * sizeof(myVec[0]));
		N_free = templateFace.MRA_index.N_R3;
		wIdx = templateFace.MRA_index.wIdx3;
		break;
	}
	case Full_Resolution:
	{
		N = templateFace.N_v;
		N_FV = N - templateFace.MRA_index.N_idxD;
		MRA_cell = templateFace.cell_ring1;
		localAdjNum = templateFace.numberRing1;

		fIdx = new int[templateFace.N_v - templateFace.MRA_index.N_idxD];
		memcpy(fIdx, templateFace.MRA_index.idxK, templateFace.MRA_index.N_idxK * sizeof(int));
		memcpy(fIdx + templateFace.MRA_index.N_idxK, templateFace.MRA_index.idxE, templateFace.MRA_index.N_idxE * sizeof(int));
		memcpy(fIdx + templateFace.MRA_index.N_idxK + templateFace.MRA_index.N_idxE, templateFace.MRA_index.idxF, templateFace.MRA_index.N_idxF * sizeof(int));
		std::vector<int> myVec(fIdx, fIdx + templateFace.N_v - templateFace.MRA_index.N_idxD);
		sort(myVec.begin(), myVec.end());
		memcpy(fIdx, &myVec[0], myVec.size() * sizeof(myVec[0]));
		N_free = templateFace.MRA_index.N_R4;
		wIdx = templateFace.MRA_index.wIdx4;
		break;
	}

	default:
		cout << "Invalid selection\n";
		exit(-1);
	}

	Dfield_T = Eigen::MatrixXd::Zero(N, 3);
	Dfield_M = Eigen::MatrixXd::Zero(N - N_FV, 3);

	Eigen::MatrixXd	residualError = Eigen::MatrixXd::Zero(N_free, 3);
	//int maxIter0 = 10;
	for (i = 0; i < maxIter; i++)
	{
		for (j = 0; j < N; j++)
		{
			Eigen::MatrixXd	pointsRmp = MRA_cell[j];
			Eigen::MatrixXd	pointsTmp = Eigen::MatrixXd::Zero(localAdjNum[j], 3);
			if ((flag == Full_Resolution || flag == Resolution_3) && binary_search(templateFace.MRA_index.idxF, templateFace.MRA_index.idxF + templateFace.MRA_index.N_idxF, j))
				continue;
			if (flag == Full_Resolution || flag == Resolution_3)
			{
				for (int jj = 0; jj < pointsTmp.rows(); jj++)
				{
					pointsTmp.row(jj) = this->points.row(templateFace.vertexRing1Idx[j][jj]) - this->points.row(j);
				}
			}
			else
			{
				for (int jj = 0; jj < pointsTmp.rows(); jj++)
				{
					pointsTmp.row(jj) = this->points.row(pIdx[MRA_vertexIdx[j][jj]]) - this->points.row(pIdx[j]);
				}
			}


			Eigen::MatrixXd RMean = pointsRmp.colwise().mean();

			pointsRmp = pointsRmp - RMean.replicate(pointsRmp.rows(), 1);
			Eigen::MatrixXd TMean = pointsTmp.colwise().mean();

			pointsTmp = pointsTmp - TMean.replicate(pointsTmp.rows(), 1);
			Eigen::MatrixXd cVar = pointsRmp.transpose()*pointsTmp;

			JacobiSVD<MatrixXd> svd(cVar, ComputeThinU | ComputeThinV);
			Eigen::MatrixXd R_rot = svd.matrixV()*svd.matrixU().transpose();
			Eigen::Vector3d V_3D(1, 1, R_rot.determinant());
			R_rot = svd.matrixV()*V_3D.asDiagonal()*svd.matrixU().transpose();
			Dfield_T.row(j) = TMean - RMean*R_rot.transpose();
		}
		//if (flag == Full_Resolution)
		//{
		//	Eigen::MatrixXd Dfield_T_tmp = Dfield_T;
		//	for (j = 0; j < N_FV; j++)
		//	{
		//		if (binary_search(templateFace.MRA_index.idxF, templateFace.MRA_index.idxF + templateFace.MRA_index.N_idxF, fIdx[j]))
		//			continue;
		//		Dfield_T.row(fIdx[j]) = localAdjNum[fIdx[j]] * Dfield_T_tmp.row(fIdx[j]);
		//		for (jj = 0; jj < localAdjNum[fIdx[j]]; jj++)
		//		{
		//			Dfield_T.row(templateFace.vertexRing1Idx[fIdx[j]][jj]) = Dfield_T.row(templateFace.vertexRing1Idx[fIdx[j]][jj])-0.5*Dfield_T_tmp.row(fIdx[j]);

		//		}

		//	}
		//}
		// added subsequently for landmarks bendings
		//if (flag == Full_Resolution)
		//{
		//	Eigen::MatrixXd Dfield_T_tmp = Dfield_T;
		//	for (j = 0; j < N_FV; j++)
		//	{
		//		if (binary_search(templateFace.MRA_index.idxF, templateFace.MRA_index.idxF + templateFace.MRA_index.N_idxF, fIdx[j]))
		//			continue;
		//		Dfield_T.row(fIdx[j]) = Dfield_T_tmp.row(fIdx[j]);
		//	}
		//}



		Dfield_M = A_Re_Matrix*Dfield_T;
		spSolver.iterativeSolve(Dfield_M.data());
		Dfield_M = Map<MatrixXd>(spSolver.x, N - N_FV, 3);
		int jj = 0;
		if (flag == Full_Resolution || flag == Resolution_3)
		{
			for (j = 0; j < N; j++)
			{
				if (jj < N_FV && fIdx[jj] == j)
					jj++;
				else
					Dfield.row(j) = Dfield_M.row(j - jj);

			}

		}
		else
		{
			for (j = 0; j < N; j++)
			{
				if (jj < N_FV && fIdx[jj] == j)
					jj++;
				else
					Dfield.row(pIdx[j]) = Dfield_M.row(j - jj);

			}
		}

		//cout << Dfield_T.row(21018).norm() << endl;

		Eigen::MatrixXd DFactor = (Dfield.array()*this->normals.array()).matrix().rowwise().sum().replicate(1, 3);
		Dfield = Dfield - (this->normals.array()*DFactor.array()).matrix();
		
		Eigen::MatrixXd tmp_points = this->points;

		this->points = this->points + Dfield;

		Eigen::VectorXd sqrD;
		Eigen::VectorXi I;
		Eigen::MatrixXd C;
		this->tree.squared_distance(OriginalTarget, this->triList, this->points, sqrD, I, C);

		for (int j = 0; j < N_free; j++)
		{
			residualError.row(j) = C.row(wIdx[j]) - tmp_points.row(wIdx[j]);
		}

		ArrayXd residualErrorArray = residualError.col(0).array()*residualError.col(0).array() + residualError.col(1).array()*residualError.col(1).array() + residualError.col(2).array()*residualError.col(2).array();
		residualErrorArray = residualErrorArray.sqrt();


		//cout << residualErrorArray.mean() << endl;
		//system("pause");

		this->points = C;
		this->updateNormals();

		/*examples*/
		//Eigen::MatrixXd e(2, 2);
		//e << 8.0, std::nan("1"), std::numeric_limits <double> ::infinity(), 4.0;
		//std::cout << e << std::endl;
		//cout << e.array().isFinite() << endl;
		//e = (e.array().isFinite()).select(e, 0.0);
		//std::cout << e << std::endl;
		//system("pause");

		/*string outputPlyFile = ".\\convergence_MR_all\\" + to_string(flag) + "_" + to_string(i) + ".ply";*/
		//string outputPlyFile = ".\\output10\\" + to_string(i+1) + ".ply";
		//this->writePlyFile(outputPlyFile);

		if (residualErrorArray.mean() < 0.001)
		{
			cout << i << endl;
			break;
		}



		//system("pause");
		//if (flag == Full_Resolution)
		//{
		//	string outputPlyFile = ".\\convergence_MR4\\" + to_string(i) + ".ply";
		//	this->writePlyFile(outputPlyFile);
		//}

	}
	//cout << "----------------------" << endl;
}

void PointCloud::loadVertexWeights(string FName)
{
	loadWeightFile(FName, &vertexWeights);
}

// correct landmarks
void PointCloud::rectifyLandmarks(string FNameIdx, string FName)
{

	ifstream ifile;
	string s;
	int count = 0;
	int *landmarkIdx;
	double *landmarks;

	int numberOfLandmarks = loadIdxFile(FNameIdx, &landmarkIdx);
	landmarks = new double[3 * numberOfLandmarks];

	ifile.open(FName.c_str(), ifstream::in);
	if (!ifile.is_open())
	{
		cout << "Unable to open point file '" << FName << "'" << endl;
		exit(-1);
	}
	getline(ifile, s);
	istringstream ss(s);
	string sTmp;
	while (ss >> sTmp)
		landmarks[count++] = atof(sTmp.c_str());


	MatrixXd AllLandmarks = Map<MatrixXd>(landmarks, numberOfLandmarks, 3);
	Eigen::VectorXd sqrD;
	Eigen::VectorXi I;
	Eigen::MatrixXd C;
	this->tree.squared_distance(points, triList, AllLandmarks, sqrD, I, C);
	for (int i = 0; i < numberOfLandmarks; i++)
	{
		//cout << points.row(landmarkIdx[i]) << endl;
		points.row(landmarkIdx[i]) = C.row(i);
		//cout << C.row(i) << endl;
		//system("pause");
	}
	this->updateNormals();
	delete(landmarkIdx);
	delete(landmarks);
	sqrD.resize(0);
	I.resize(0);
	C.resize(0, 0);
	//return;
}



MRA_PCloud::MRA_PCloud()
{
	N0 = 0;
	N1 = 0;
	N2 = 0;
	F0 = 0;
	F1 = 0;
	F2 = 0;
	M0 = 0;
	M1 = 0;
	M2 = 0;
	pIdx0 = NULL;
	pIdx1 = NULL;
	pIdx2 = NULL;
	fIdx0 = NULL;
	fIdx1 = NULL;
	fIdx2 = NULL;
	mIdx0 = NULL;
	mIdx1 = NULL;
	mIdx2 = NULL;
}

void MRA_PCloud::loadMRA_0(string FName1, string FName2, string FName3)
{
	N0 = loadIdxFile(FName1, &pIdx0);
	F0 = loadIdxFile(FName2, &fIdx0);
	M0 = loadIdxFile(FName3, &mIdx0);
	M0 /= 2;//half of the vertice number is the edge number
}

void MRA_PCloud::loadMRA_1(string FName1, string FName2, string FName3)
{
	N1 = loadIdxFile(FName1, &pIdx1);
	F1 = loadIdxFile(FName2, &fIdx1);
	M1 = loadIdxFile(FName3, &mIdx1);
	M1 /= 2;
}

void MRA_PCloud::loadMRA_2(string FName1, string FName2, string FName3)
{
	N2 = loadIdxFile(FName1, &pIdx2);
	F2 = loadIdxFile(FName2, &fIdx2);
	M2 = loadIdxFile(FName3, &mIdx2);
	M2 /= 2;
}

void MRA_PCloud::loadFreeIndices(string FName1, string FName2, string FName3, string FName4, string FName5)
{
	N_R0 = loadIdxFile(FName1, &wIdx0);
	N_R1 = loadIdxFile(FName2, &wIdx1);
	N_R2 = loadIdxFile(FName3, &wIdx2);
	N_R3 = loadIdxFile(FName4, &wIdx3);
	N_R4 = loadIdxFile(FName5, &wIdx4);
}


void MRA_PCloud::loadIdxCatogory(string FName1, string FName2, string FName3, string FName4)
{
	N_idxK = loadIdxFile(FName1, &idxK);
	N_idxE = loadIdxFile(FName2, &idxE);
	N_idxF = loadIdxFile(FName3, &idxF);
	N_idxD = loadIdxFile(FName4, &idxD);
}



//void PointCloud::loadFixedPointIdx(string FName)
//{
//	//ifstream ifile;
//	//string s;
//	//int count = 0;
//
//	//ifile.open(FName.c_str(), ifstream::in);
//	//if (!ifile.is_open())
//	//{
//	//	cout << "Unable to open point file '" << FName << "'" << endl;
//	//	exit(-1);
//	//}
//	//getline(ifile, s);
//	//istringstream ss1(s);
//	//string sTmp;
//	//while (ss1 >> sTmp)
//	//	count++;
//	//istringstream ss2(s);
//	//fixedPointIdx = new int[count];
//	//count = 0;
//	//while (ss2 >> sTmp)
//	//	fixedPointIdx[count++] = atoi(sTmp.c_str());
//	//N_k = count;
//	N_k = loadIdxFile(FName, &fixedPointIdx);
//}
//
//void PointCloud::loadlandmarkIdx(string FName)
//{
//	//ifstream ifile;
//	//string s;
//	//int count = 0;
//
//	//ifile.open(FName.c_str(), ifstream::in);
//	//if (!ifile.is_open())
//	//{
//	//	cout << "Unable to open point file '" << FName << "'" << endl;
//	//	exit(-1);
//	//}
//	//getline(ifile, s);
//	//istringstream ss1(s);
//	//string sTmp;
//	//while (ss1 >> sTmp)
//	//	count++;
//	//istringstream ss2(s);
//	//landmarkIdx = new int[count];
//	//count = 0;
//	//while (ss2 >> sTmp)
//	//	landmarkIdx[count++] = atoi(sTmp.c_str());
//	//N_l = count;
//	N_l = loadIdxFile(FName, &landmarkIdx);
//}
