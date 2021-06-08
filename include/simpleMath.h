// -*- coding: utf-8 -*-
// -*- lsst - c++ - *-

/**
 * @author : % (Arturo Bertomeu-Motos)
 * @email : % (arturobm90@gmail.com)
 * @institution : % (Biomedical Neuroengineering Research Group (UMH) (https://nbio.umh.es/))
 */

#ifndef SIMPLEMATH_H
#define SIMPLEMATH_H

#include <iostream>

#define _USE_MATH_DEFINES
#include <math.h>

#define EPS 1e-10

class Matrix {
public:
	/// Constructors
	Matrix(int r, int c, double *data);
	Matrix(int r, int c);
	Matrix(int r, double *data);
	Matrix(int r);
	Matrix();
	Matrix(const Matrix&);
	Matrix(const Matrix*);
	/// Destructor
	~Matrix();

	/// GET functions
	int		getRows() { return this->rows; };
	int		getCols() { return this->cols; };

	/// SET functions
	int		setDimension(int r, int c);
	int		setDimension(int r);

	/// STANDARD Matrix
	int		Zeros();
	int		Identity();
	int		Ones();
	int		Rand(); //TODO

	/// Math Operations MATRIX
	int		Transpose();
	int		Transpose(Matrix *outMat);
	double	Trace();

	int		Add(Matrix* outMat, Matrix*m);
	int		Add(Matrix*m);
	int		Subs(Matrix* outMat, Matrix*m);
	int		Subs(Matrix*m);
	int		Mult(Matrix* outMat, Matrix*m);
	int		Mult(Matrix*m);

	int		Mult(Matrix* outMat, double n);
	int		Mult(double n);
	int		Division(Matrix* outMat, double n);
	int		Division(double n);
	int		Exp(Matrix* outMat, int n); //TODO
	int		Exp(int n); //TODO

	int		Inv3x3();
	int		Inv3x3(Matrix* outMat);
	int		Inv4x4();
	int		Inv4x4(Matrix* outMat);
	int		Inverse();
	int		Inverse(Matrix* outMat);

	/// Help inverse
	int		augment(Matrix *B);
	int		augment(Matrix *AB, Matrix *B);
	int		gaussianEliminate();
	int		gaussianEliminate(Matrix* outMat);
	int		rowReduceFromGaussian();
	int		rowReduceFromGaussian(Matrix* outMat);
	int		swapRows(int r1, int r2);

	/// Operators
	inline double& operator()(int x, int y) { return mat[x][y]; }
	Matrix& operator=(const Matrix&);

	friend std::ostream& operator<<(std::ostream&, const Matrix&);
	friend std::ostream& operator<<(std::ostream&, const Matrix*);
	friend std::istream& operator>>(std::istream&, Matrix&);

private:
	int rows, cols;
	double **mat;

	int invSqrt(float *out, float x);
	void allocSpaceMat();
	void deleteSpaceMat();

};

class Vector {

public:
	/// Constructor
	Vector(int l, double *data);
	Vector(int l);
	Vector();
	Vector(const Vector&);
	Vector(const Vector*);
	/// Destructor
	~Vector();

	/// GET functions
	int getLength() {
		if (rows == 1)
			return this->cols;
		else
			return this->rows;
	};
	bool	isRow();
	bool	isCol();

	/// SET functions
	int		setLength(int l);

	/// STANDARD Vector
	int		Zeros();
	int		Ones();
	int		Rand(); //TODO

	/// Math Operations Vector
	int		Transpose(Vector *outMat);
	int		Transpose();
	int		Add(Vector* outVec, Vector* v);
	int		Add(Vector* v);
	int		Subs(Vector* outVec, Vector* v);
	int		Subs(Vector* v);
	int		Mult(Vector *v);							// V*V->V
	int		Mult(Vector *vout, Vector *v);				// V*V->V
	int		Mult(Matrix *mout, Vector *v);				// V*V->M 
	int		Mult(Vector *vout, Matrix *m);				// V*M->V 
	int		Mult(Matrix *m);							// V*M->V 
	int		Mult(Matrix *mout, Matrix *m);				// V*M->M
	int		ElementMult(Vector* outVec, Vector* v);
	int		ElementMult(Vector* v);

	int		Mult(Vector* outVec, double n);
	int		Mult(double n);
	int		Division(Vector* outVec, double n);
	int		Division(double n);
	int		ElementExp(Vector* outVec, int n); //TODO
	int		ElementExp(int n); //TODO

	/// Math Operations VECTORS
	int		CrossProd(Vector *outVec, Vector *m);
	int		CrossProd(Vector *m);
	double	Norm();
	double	DotProd(Vector* v);
	int		VecNorm(Vector *outVec);
	int		VecNorm();

	/// Conversions
	int		vec2mat(Matrix *m);
	int		mat2vec(Matrix *m);

	/// Operators
	inline double& operator()(int x) {
		if (rows == 1)
			return vec[0][x];
		else
			return vec[x][0];
	}

	Vector& operator=(const Vector&);
	friend std::ostream& operator<<(std::ostream&, const Vector&);
	friend std::ostream& operator<<(std::ostream&, const Vector*);
	friend std::istream& operator>>(std::istream&, Vector&);

private:
	int rows, cols;
	double **vec;

	int		invSqrt(float *out, float x);

	void	allocSpaceVec();
	void	deleteSpaceVec();
};

class Quaternion {

public:

	struct QuaternionStruct {
		QuaternionStruct() : qs(1), qx(0), qy(0), qz(0) {}
		double		qs;
		double		qx;
		double		qy;
		double		qz;
	};

	/// Constructor
	Quaternion(QuaternionStruct *q);
	Quaternion(double qs, double qx, double qy, double qz);
	Quaternion();
	Quaternion(const Quaternion&);
	Quaternion(const Quaternion*);
	/// Destructor
	~Quaternion();

	/// STANDARD Quaternion
	int		Identity();

	/// Update Quaternion
	int		Update(QuaternionStruct *q);
	int		Update(double qs, double qx, double qy, double qz);

	/// Math Operations
	int		Norm(Quaternion *outQuat);
	int		Norm();
	int		Conj(Quaternion *outQuat);
	int		Conj();
	int		Prod(Quaternion *outQuat, Quaternion *q);
	int		Prod(Quaternion *q);

	// Transformations
	Matrix	q2tr();
	Matrix	q2r();
	Vector	q2Euler();			//(Roll-Pitch-Yaw)(XYZ)
	int		r2q(Matrix *m);     //TODO


	// Operators
	inline double& operator()(int x) { return quat[x]; }
	Quaternion& operator=(const Quaternion&);

	friend std::ostream& operator<<(std::ostream&, const Quaternion&);
	friend std::ostream& operator<<(std::ostream&, const Quaternion*);
	friend std::istream& operator>>(std::istream&, Quaternion&);

private:
	double			*quat;
	const double	quat_size = 4;

	void	allocSpaceQuat();
	void	deleteSpaceQuat();
	int		invSqrt(float *out, float x);

};

#endif // SIMPLEMATH_H

