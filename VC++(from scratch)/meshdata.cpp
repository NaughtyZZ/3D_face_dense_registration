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

// sort indices
template <typename T>
vector<int> sort_indexes(const vector<T> &v) {

	//initialize original index locations
	vector<int> idx(v.size());
	for (int i = 0; i != idx.size(); ++i) idx[i] = i;

	//sort indexes based on comparing values in v
	sort(idx.begin(), idx.end(),
		[&v](int i1, int i2) {return v[i1] < v[i2]; });

	return idx;
}



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
// initialize
PointCloud::PointCloud()
{
	N_v = 0;
	N_f = 0;
	N_k = 0;

	//N_k = 0;
	//N_l = 0;
	this->points(N_v, 3);
	this->normals(N_v, 3);
	this->textures(N_v, 3);
	this->triList(N_f, 3);
	this->vertexRing1Idx = NULL;
	this->numberRing1 = NULL;
	this->cell_ring1 = NULL;
	this->landmarks = NULL;

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
	igl::per_vertex_normals(this->points, this->triList, this->normals);

	this->normals = (this->normals.array().isFinite()).select(this->normals, 0.0);





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
	this->tree.init(points, triList);
}



void PointCloud::loadLandmarksIdx(string FName)
{
	this->N_k = loadIdxFile(FName, &this->landmarks);
	int i = 0;
	this->landmarkPositions = Eigen::MatrixXd::Zero(this->N_k, 3);
	for (i = 0; i < this->N_k; i++)
	{
		this->landmarkPositions.row(i) = this->points.row(this->landmarks[i]);
	}
}


void PointCloud::computeOuterEdgeIdx()
{
	int i, j;


	//convert triangle to edges
	Eigen::MatrixXi allEdges(3 * this->N_f, 2);
	allEdges.topRows(this->N_f) = this->triList.leftCols(2);
	allEdges.middleRows(this->N_f, this->N_f) = this->triList.rightCols(2);
	allEdges.bottomLeftCorner(this->N_f, 1) = this->triList.col(2);
	allEdges.bottomRightCorner(this->N_f, 1) = this->triList.col(0);

	//for (i = 2*this->N_f; i < 2*this->N_f+20; i++)
	//{
	//cout << allEdges.row(i) <<endl;
	//}
	//system("pause");


	//sort each row
	for (i = 0; i < allEdges.rows(); i++)
	{
		if (allEdges(i, 0) > allEdges(i, 1))
		{
			int tmp = allEdges(i, 0);
			allEdges(i, 0) = allEdges(i, 1);
			allEdges(i, 1) = tmp;
		}
	}

	// compute sort indices for the first column
	int *edgesIdx1 = new int[allEdges.rows()];
	for (i = 0; i < allEdges.rows(); i++)
		edgesIdx1[i] = allEdges(i, 0);
	std::vector<int> myVec(edgesIdx1, edgesIdx1 + allEdges.rows());
	vector<int> sortIdx = sort_indexes(myVec);

	//compute row range in sorted rows for the specified edges that share the same vertices
	//compute mesh boundary according to the principle 'a boundary edge is not shared by two triangles'.
	int *rangeStart = new int[allEdges.rows()];
	int *rangeEnd = new int[allEdges.rows()];
	for (i = 0; i < allEdges.rows(); i++)
	{
		if (i == 0)
		{
			rangeStart[i] = 0;
			rangeEnd[i] = rangeStart[i];
			for (j = i; j < allEdges.rows(); j++)
			{
				if (allEdges(sortIdx[i], 0) != allEdges(sortIdx[j], 0))
					break;
				else
					rangeEnd[i]++;
			}

		}
		else if (i == allEdges.rows() - 1)
		{
			if (allEdges(sortIdx[i], 0) == allEdges(sortIdx[i - 1], 0))
			{
				rangeStart[i] = rangeStart[i - 1];
				rangeEnd[i] = rangeEnd[i - 1];
			}
			else
				rangeStart[i] = allEdges.rows() - 1;
			rangeEnd[i] = allEdges.rows();


		}
		else
		{
			if (allEdges(sortIdx[i], 0) == allEdges(sortIdx[i - 1], 0))
			{
				rangeStart[i] = rangeStart[i - 1];
				rangeEnd[i] = rangeEnd[i - 1];
			}
			else
			{
				rangeStart[i] = rangeEnd[i - 1];
				rangeEnd[i] = rangeStart[i];
				for (j = i; j < allEdges.rows(); j++)
				{
					if (allEdges(sortIdx[i], 0) != allEdges(sortIdx[j], 0))
						break;
					else
						rangeEnd[i]++;
				}
			}
		}

	}


	//for (i = 0; i < 20; i++)
	//{
	//cout << allEdges.row(sortIdx[i])<< ' '<< rangeStart[i] << ' ' << rangeEnd[i] <<endl;
	//}
	//system("pause");


	// correspondence of triangle and edge indices 
	int count = 0;
	int *edgeTrianglesTmp = new int[this->N_v];
	Eigen::MatrixXi OuterEdges(this->N_v, 2);

	for (i = 0; i < allEdges.rows(); i++)
	{
		int flag = 0;
		for (j = rangeStart[i]; j < rangeEnd[i]; j++)
		{
			if (i != j &&allEdges(sortIdx[j], 1) == allEdges(sortIdx[i], 1))
			{
				flag = 1;
				break;
			}

		}
		if (flag == 0)
		{
			//cout << allEdges.row(sortIdx[i]) << ' ' << (int)(sortIdx[i] / 3) << ' ' << this->triList.row(sortIdx[i] % this->N_f) << endl;
			//system("pause");
			edgeTrianglesTmp[count] = sortIdx[i] % this->N_f;
			OuterEdges.row(count) = allEdges.row(sortIdx[i]);
			count++;


		}
	}
	//cout <<count << endl;
	/*for (i = 0; i < count; i++)
	cout << edgeTrianglesTmp [i] << ' ' << OuterEdges.row(i)<< endl;*/


	// compute non-repetitive triangle indices
	std::vector<int> myVec2(edgeTrianglesTmp, edgeTrianglesTmp + count);
	vector<int> sortIdx2 = sort_indexes(myVec2);
	sort(myVec2.begin(), myVec2.end());
	myVec2.erase(unique(myVec2.begin(), myVec2.end()), myVec2.end());

	// initialization of output triangle indices, edges, and flags of 1 or 2 edges
	this->edgeTriangles = new int[myVec2.size()];
	this->edgesVerticesInTriangles = -Eigen::MatrixXi::Ones(myVec2.size(), 4);
	this->flagsForOuterEdges = new int[myVec2.size()];
	memset(flagsForOuterEdges, 0, myVec2.size() * sizeof(int));


	//compute mesh boundary according to the principle 'a boundary edge is not shared by two triangles'.
	int count2 = 0;
	for (i = 0; i < count; i++)
	{
		if (i == 0)
		{
			this->edgeTriangles[count2] = edgeTrianglesTmp[sortIdx2[i]];
			this->edgesVerticesInTriangles.block(count2, 0, 1, 2) = OuterEdges.row(sortIdx2[i]);
		}
		else if (edgeTrianglesTmp[sortIdx2[i]] != edgeTrianglesTmp[sortIdx2[i - 1]])
		{
			count2++;
			this->edgeTriangles[count2] = edgeTrianglesTmp[sortIdx2[i]];
			this->edgesVerticesInTriangles.block(count2, 0, 1, 2) = OuterEdges.row(sortIdx2[i]);
		}
		else
		{
			this->flagsForOuterEdges[count2] = 1;
			this->edgesVerticesInTriangles.block(count2, 2, 1, 2) = OuterEdges.row(sortIdx2[i]);
		}


	}
	//cout << count<<' '<<myVec2.size() << endl;
	//for (i = 0; i < myVec2.size(); i++)
	//{
	//	cout << edgeTriangles[i]<<' '<< edgesVerticesInTriangles.row(i)<<' ' << flagsForOuterEdges[i]<<endl;
	//}

}


//load weights for initial deformation, each weight is precomputed being invesely proportional to the geodesic distance between each vertex and each landmark
void PointCloud::loadWeightsFromDfield(string FName)
{
	int i;
	ifstream ifile;
	string s1, s2;
	ifile.open(FName.c_str(), ifstream::in);
	if (!ifile.is_open())
	{
		cout << "Unable to open point file '" << FName << "'" << endl;
		exit(-1);
	}

	this->weightsForLandmarks = Eigen::MatrixXd::Zero(this->N_v, this->N_k);


	for (i = 0; i < this->N_v; i++)
	{
		getline(ifile, s1);
		istringstream ss1(s1);
		int count = 0;
		while (ss1 >> s2)
		{
			this->weightsForLandmarks(i, count++) = atof(s2.c_str());
		}

	}
	ifile.close();
}



void PointCloud::loadLandmarkPositions(string FName)
{
	int i;
	ifstream ifile;
	string s1, s2, s3, s4;
	ifile.open(FName.c_str(), ifstream::in);
	if (!ifile.is_open())
	{
		cout << "Unable to open point file '" << FName << "'" << endl;
		exit(-1);
	}
	int count = 0;
	while (getline(ifile, s1))
	{
		count++;
	}
	ifile.close();

	this->landmarkPositions = Eigen::MatrixXd::Zero(count, 3);

	ifile.open(FName.c_str(), ifstream::in);
	if (!ifile.is_open())
	{
		cout << "Unable to open point file '" << FName << "'" << endl;
		exit(-1);
	}
	for (i = 0; i < count; i++)
	{
		getline(ifile, s1);
		istringstream ss1(s1);

		ss1 >> s2 >> s3 >> s4;
		this->landmarkPositions(i, 0) = atof(s2.c_str());
		this->landmarkPositions(i, 1) = atof(s3.c_str());
		this->landmarkPositions(i, 2) = atof(s4.c_str());
	}
	ifile.close();
}



//compute adjacenty of 1-ring neighbors
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

//initial deformation according to CVPR-19 (fan et al. 'Boosting local shape matching for dense 3D face correspondence') 
void PointCloud::initialDeformation(PointCloud targetFace)
{
	int i, j;
	Eigen::MatrixXd deformedTemplate = Eigen::MatrixXd::Zero(this->N_v, 3);

	for (j = 0; j < this->N_v; j++)
	{
		Eigen::MatrixXd	pointsRmp = this->landmarkPositions;
		Eigen::MatrixXd	pointsTmp = targetFace.landmarkPositions;

		Eigen::MatrixXd weights = this->weightsForLandmarks.row(j).replicate(3, 1).transpose();


		/*Eigen::MatrixXd RMean = pointsRmp.colwise().mean();
		Eigen::MatrixXd TMean = pointsTmp.colwise().mean();*/


		Eigen::MatrixXd RMean = pointsRmp.cwiseProduct(weights).colwise().sum() / this->weightsForLandmarks.row(j).sum();
		pointsRmp = pointsRmp - RMean.replicate(pointsRmp.rows(), 1);

		Eigen::MatrixXd TMean = pointsTmp.cwiseProduct(weights).colwise().sum() / this->weightsForLandmarks.row(j).sum();
		pointsTmp = pointsTmp - TMean.replicate(pointsTmp.rows(), 1);

		Eigen::MatrixXd cVar = pointsRmp.transpose()*this->weightsForLandmarks.row(j).asDiagonal()*pointsTmp;

		//weighted rigid transformation between corresponding landmarks
		JacobiSVD<MatrixXd> svd(cVar, ComputeThinU | ComputeThinV);
		Eigen::MatrixXd R_rot = svd.matrixV()*svd.matrixU().transpose();
		Eigen::Vector3d V_3D(1, 1, R_rot.determinant());
		R_rot = svd.matrixV()*V_3D.asDiagonal()*svd.matrixU().transpose();
		Eigen::MatrixXd trans = TMean - RMean*R_rot.transpose();
		deformedTemplate.row(j) = this->points.row(j)*R_rot.transpose() + trans;
	}

	//setting exact landmark positions
	for (j = 0; j < this->N_k; j++)
	{
		deformedTemplate.row(this->landmarks[j]) = targetFace.landmarkPositions.row(j);
	}


	//update deformed template
	this->points = deformedTemplate;

}




//main block for the registration process according to IJCV 2023 fan et al 'Towards Fine-Grained Optimal 3D Face Dense Registration: An Iterative Dividing and Diffusing Method'
void PointCloud::denseRegistration(PointCloud targetFace, mkl_eq_solver spSolver, SpMat A_Re_Matrix, int maxIter)
{
	int i, j, jj;
	Eigen::MatrixXd Dfield;  //total offset
	Eigen::MatrixXd Dfield1; //normal offset
	Eigen::MatrixXd Dfield2; //tangential offset
	Eigen::MatrixXd Dfield_M; //offset for free vertices
	Eigen::VectorXd sqrD;
	Eigen::VectorXi I; //indices of nearest triangles
	Eigen::MatrixXd C; //nearest locations in triangles

	//sort landmark indices
	std::vector<int> fIdx(this->landmarks, this->landmarks + this->N_k);
	sort(fIdx.begin(), fIdx.end());


	this->updateNormals();

	for (i = 0; i < maxIter; i++)
	{
		// AABB nearest points computation
		targetFace.tree.squared_distance(targetFace.points, targetFace.triList, this->points, sqrD, I, C);
		Dfield1 = C - this->points;

		// outlier principle 1: the distance to closest vertex is above a threshold (we set to 5 here)
		Eigen::MatrixXd Dis = Dfield1.rowwise().norm();
		int cc = 0; //count for outliers
		int *outlierIdx = new int[this->N_v];
		memset(outlierIdx, -1, this->N_v * sizeof(int));
		for (j = 0; j < this->N_v; j++)
		{
			if (Dis(j) > 5)
			{
				Dfield1.row(j) = Dfield1.row(j) - Dfield1.row(j);
				outlierIdx[cc] = j;
				cc++;
			}
		}
		//cout <<cc << endl;



		Dfield2 = Eigen::MatrixXd::Zero(this->N_v, 3);

		//for (int k = 0; k < 100; k++)
		//	cout << targetFace.edgeTriangles[k] <<' '<< targetFace.edgesVerticesInTriangles.row(k)<<' '<<targetFace.flagsForOuterEdges[k]<< endl;
		//system("pause");
		//std::vector<int> myVec(targetFace.edgeTriangles, targetFace.edgeTriangles + targetFace.edgesVerticesInTriangles.rows());


		// outlier principle 2: the closest vertex is on the boundary of the target mesh
		for (j = 0; j < this->N_v; j++)
		{
			if (binary_search(targetFace.edgeTriangles, targetFace.edgeTriangles + targetFace.edgesVerticesInTriangles.rows(), I(j)))//outlier principle 2
			{
				outlierIdx[cc] = j;
				cc++;


				int k; //indices of outliers in triangle
				for (k = 0; k < targetFace.edgesVerticesInTriangles.rows(); k++)
				{
					if (targetFace.edgeTriangles[k] == I(j))
						break;
				}



				int idx1 = targetFace.edgesVerticesInTriangles(k, 0);
				int idx2 = targetFace.edgesVerticesInTriangles(k, 1);

				/*cout << targetFace.edgesVerticesInTriangles.row(k) << endl;
				cout << k <<' '<< idx1 << ' ' << idx2 << ' ' << endl;*/

				Eigen::Vector3d Vec1 = targetFace.points.row(idx1) - targetFace.points.row(idx2);
				Eigen::Vector3d Vec2 = targetFace.points.row(idx1) - C.row(j);

				/*	cout << Vec1<< endl;
					cout << Vec2 << endl;
					system("pause");*/

				if (Vec1.cross(Vec2).norm() < 0.0000000000001) //collinear of two vectors
				{
					Dfield1.row(j) = Dfield1.row(j) - Dfield1.row(j); //setting 0
					continue;
				}



				if (targetFace.flagsForOuterEdges[k] != 0) // case of 2 outer edges in a triangle
				{
					idx1 = targetFace.edgesVerticesInTriangles(k, 2);
					idx2 = targetFace.edgesVerticesInTriangles(k, 3);

					Vec1 = targetFace.points.row(idx1) - targetFace.points.row(idx2);
					Vec2 = targetFace.points.row(idx1) - C.row(j);
					if (Vec1.cross(Vec2).norm() < 0.0000000000001) //collinear of two vectors
						Dfield1.row(j) = Dfield1.row(j) - Dfield1.row(j); //setting 0

				}

			}




		}



		// compute tangential offset for each vertices
		for (j = 0; j < this->N_v; j++)
		{




			Eigen::MatrixXd	pointsRmp = this->cell_ring1[j];
			Eigen::MatrixXd	pointsTmp = Eigen::MatrixXd::Zero(this->numberRing1[j], 3);
			/*if (binary_search(this->landmarks, this->landmarks + this->N_k, j))
				continue;*/

			for (int jj = 0; jj < pointsTmp.rows(); jj++)
			{
				pointsTmp.row(jj) = this->points.row(this->vertexRing1Idx[j][jj]) - this->points.row(j);
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
			Dfield2.row(j) = TMean - RMean*R_rot.transpose();
		}


		// normal offest in the normal direction
		Eigen::MatrixXd DFactor1 = (Dfield1.array()*this->normals.array()).matrix().rowwise().sum().replicate(1, 3);
		Dfield1 = (this->normals.array()*DFactor1.array()).matrix();


		// tangential offest in the tangential direction
		Eigen::MatrixXd Dfield2_tmp = Dfield2;
		Eigen::MatrixXd DFactor2 = (Dfield2.array()*this->normals.array()).matrix().rowwise().sum().replicate(1, 3);
		Dfield2 = Dfield2 - (this->normals.array()*DFactor2.array()).matrix();


		// offest for outliers in rigid formulation
		for (j = 0; j < cc; j++)
			Dfield2.row(outlierIdx[j]) = Dfield2_tmp.row(outlierIdx[j]);

		//total offset
		Dfield = Dfield1 + Dfield2;



		//solve least-square equations
		Dfield_M = A_Re_Matrix*Dfield;
		spSolver.iterativeSolve(Dfield_M.data());
		Dfield_M = Map<MatrixXd>(spSolver.x, this->N_v - this->N_k, 3);

		//update regularized offsets for free vertices
		int jj = 0;
		for (j = 0; j < this->N_v; j++)
		{
			if (jj < this->N_k && fIdx[jj] == j)
			{
				jj++;
				Dfield.row(j) = Dfield.row(j) - Dfield.row(j);
			}

			else
				Dfield.row(j) = Dfield_M.row(j - jj);

		}

		//update deformed mesh
		this->points = this->points + Dfield;
		this->updateNormals();

		//stopping criterion 
		if (Dfield.rowwise().norm().mean() < 0.01)
			break;



	}

	cout << "Iterations: " << i << endl;
}