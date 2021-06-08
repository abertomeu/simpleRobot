#ifndef SIMPLEROBOT_H
#define SIMPLEROBOT_H

#include <fstream>
#include <iostream>
#include <simpleMath.h>
#include <math.h>

using namespace std;

class Robot
{
public:
	// Constructor
	Robot(int DoFs, string joints, Vector *theta, Vector *alpha, Vector *d, Vector *a);
	Robot();
	~Robot() {};


	// Get Functions
	Vector* getTheta();
	double getTheta(int idx);
	Vector* getD();
	double getD(int idx);
	Vector* getA();
	double getA(int idx);
	Vector* getAlpha();
	double getAlpha(int idx);


	// Set Functions
	int setJoints(double theta, double d, double a, double alpha, char joint); // TODO

	// Generic Operations
	int denavit(Matrix *m, double theta, double d, double a, double alpha);
	int rotx(Matrix *m, double theta);
	int roty(Matrix *m, double theta);
	int rotz(Matrix *m, double theta);
	int r2t(Matrix *m);
	int t2r(Matrix *m);
	int tr2diff(Vector *v, Matrix *t1, Matrix *t2);
	int tr2diff(Vector *v, Matrix *t);

	// Robot Kinematics
	int fKine(Matrix *m, Vector *q);
	int fKineN(Matrix *m, Vector *q, int n);
	int JacobN(Matrix *m, Vector *q);
	int Jacob0(Matrix *m, Vector *q);
	int iKine(Vector *q_out, Matrix *tr, Vector *q);	// TODO
	int iKine(Vector *q_out, Matrix *tr);			// TODO

	// Operators
	friend std::ostream& operator<<(std::ostream&, const Robot&);
	friend std::ostream& operator<<(std::ostream&, const Robot*);
	Robot& operator=(const Robot&);

private:
	int DoFs;
	string Joints;

	Vector *Theta;
	Vector *Alpha;
	Vector *D;
	Vector *A;
};

//// Operators
//inline double& operator()(int x) { return quat[x]; }
//Quaternion& operator=(const Quaternion&);
//
//friend std::ostream& operator<<(std::ostream&, const Quaternion&);
//friend std::istream& operator>>(std::istream&, Quaternion&);

/* OPERATORS FUNCTIONS
********************************/
//ostream& operator<<(ostream& os, const Matrix& m)
//{
//	for (int i = 0; i < m.rows; ++i) {
//		os << m.mat[i][0];
//		for (int j = 1; j < m.cols; ++j) {
//			os << " " << m.mat[i][j];
//		}
//		os << endl;
//	}
//	return os;
//}
//
//istream& operator>>(istream& is, Matrix& m)
//{
//	for (int i = 0; i < m.rows; ++i) {
//		for (int j = 0; j < m.cols; ++j) {
//			is >> m.mat[i][j];
//		}
//	}
//	return is;
//}
//
//Matrix& Matrix::operator=(const Matrix& m)
//{
//	if (rows != m.rows || cols != m.cols) {
//
//		deleteSpaceMat();
//
//		rows = m.rows;
//		cols = m.cols;
//
//		allocSpaceMat();
//	}
//
//	for (int i = 0; i < rows; ++i) {
//		for (int j = 0; j < cols; ++j) {
//			mat[i][j] = m.mat[i][j];
//		}
//	}
//	return *this;
//}



#endif