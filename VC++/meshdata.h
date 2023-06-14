#pragma once
#include "AABB.h"
//#include <cassert>
//#include <string.h>
#include "mkl_solve_eq.h"
using namespace std;

enum MRA { Resolution_0 = 0, Resolution_1 = 1, Resolution_2 = 2, Resolution_3 = 3, Full_Resolution = 4 };

//template <typename DerivedV, 3>

int loadIdxFile(string FName, int **idxList);

class MRA_PCloud
{
public:
	// indices and numbers of landmarks, edge points, fixed point, and dynamic points, respectively
	int N_idxK, N_idxE, N_idxF, N_idxD;
	int *idxK, *idxE, *idxF, *idxD;

	// indices and numbers of free vertices within a frontal masks for different resolutions
	int N_R0, N_R1, N_R2, N_R3, N_R4;
	int *wIdx0, *wIdx1, *wIdx2, *wIdx3, *wIdx4;


	//indices and numbers referring to all vertices
	int N0, N1, N2;
	int *pIdx0, *pIdx1, *pIdx2;

	//indices and numbers referring to MRA vertices itselves
	int F0, F1, F2;
	int *fIdx0, *fIdx1, *fIdx2;

	//adjacenty index in the form of edges
	int M0, M1, M2;
	int *mIdx0, *mIdx1, *mIdx2;

	MRA_PCloud();
	// load indices and numbers of landmarks, edge points, fixed point, and dynamic points, respectively
	void loadIdxCatogory(string FName1, string FName2, string FName3, string FName4);
	// load indices and numbers for MRA 0, 1, and 2, respectively
	void loadMRA_0(string FName1, string FName2, string FName3);
	void loadMRA_1(string FName1, string FName2, string FName3);
	void loadMRA_2(string FName1, string FName2, string FName3);
	void loadFreeIndices(string FName1, string FName2, string FName3, string FName4, string FName5);

};

class PointCloud
{
public:
	//number of vertices and triangles
	int N_v, N_f;
	Eigen::MatrixXd points;
	Eigen::MatrixXd normals;
	Eigen::MatrixXi textures;
	Eigen::MatrixXi triList;

	double *vertexWeights;



	//Adjacenty of 1-ring
	int *numberRing1;
	int **vertexRing1Idx;
	Eigen::MatrixXd *cell_ring1;

	//MRA indices
	MRA_PCloud MRA_index;
	//Resolution 0 for this template
	int *MRA_N0;
	int **MRA_vertexIdx0;
	Eigen::MatrixXd *MRA_cell0;
	//Resolution 1 for this template
	int *MRA_N1;
	Eigen::MatrixXd *MRA_cell1;
	int **MRA_vertexIdx1;
	//Resolution 2 for this template
	int *MRA_N2;
	Eigen::MatrixXd *MRA_cell2;
	int **MRA_vertexIdx2;

	//AABB tree for closest point computation
	igl::AABB<Eigen::MatrixXd, 3> tree;


	//double *weights;
	PointCloud();
	void updateNormals();
	void buildAABBTree();
	void loadPlyFile(string FName);
	void writePlyFile(string FName);
	void rectifyLandmarks(string FNameIdx, string FName);

	void computeVertexAndCellRing1();
	void computeMRAindex();
	void rectifyMRAVertices(PointCloud templateFace, mkl_eq_solver spSolver, Eigen::MatrixXd OriginalTarget, SpMat A_Re_Matrix, int maxIter, MRA flag);
	void loadVertexWeights(string FName);
};

