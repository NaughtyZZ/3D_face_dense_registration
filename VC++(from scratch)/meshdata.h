#pragma once
#include "AABB.h"
#include "mkl_solve_eq.h"
using namespace std;

int loadIdxFile(string FName, int **idxList);

class PointCloud
{
public:

	int N_v, N_f; //number of vertices N_v and triangles N_f
	Eigen::MatrixXd points; // vertices
	Eigen::MatrixXd normals;
	Eigen::MatrixXi textures;
	Eigen::MatrixXi triList; // triangles


	int N_k; //number of landmarks
	int *landmarks; //landmarks indices
	Eigen::MatrixXd landmarkPositions;
	Eigen::MatrixXd weightsForLandmarks; //weights for initial deformation according to the geodesic distances from each vertex to each landmark


	//for outer edge vertices in nearest neighboring searching
	int *edgeTriangles; //indices for edgeTriangles
	Eigen::MatrixXi edgesVerticesInTriangles; //edges in the corresponding triangles in 'edgeTriangles'
	int *flagsForOuterEdges; //0 for 1 edges, 1 for two edges in 'edgesVerticesInTriangles'




	//Adjacenty of 1-ring
	int *numberRing1; // number of each cell
	int **vertexRing1Idx; // vertex indices of each cell
	Eigen::MatrixXd *cell_ring1; // matrices for each cell


	//AABB tree for point-to-surface closest point computation
	igl::AABB<Eigen::MatrixXd, 3> tree;


	PointCloud();
	void updateNormals();
	void buildAABBTree();
	void computeOuterEdgeIdx(); //compute boundary of mesh
	void loadPlyFile(string FName);
	void writePlyFile(string FName);
	void loadLandmarksIdx(string FName);
	void loadWeightsFromDfield(string FName); //load weights for initial deformation
	void loadLandmarkPositions(string FName);


	void computeVertexAndCellRing1(); //compute adjacency relationship of all vertices

	void initialDeformation(PointCloud targetFace); // initial deformation according to the method [CVPR 19'Boosting local shape matching for dense 3D face correspondence] 

	void denseRegistration(PointCloud targetFace, mkl_eq_solver spSolver, SpMat A_Re_Matrix, int maxIter); // the main block of the proposed method

};

