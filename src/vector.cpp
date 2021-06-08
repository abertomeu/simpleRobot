// -*- coding: utf-8 -*-
// -*- lsst - c++ - *-

/**
 * @author : % (Arturo Bertomeu-Motos)
 * @email : % (arturobm90@gmail.com)
 * @institution : % (Biomedical Neuroengineering Research Group (UMH) (https://nbio.umh.es/))
 */

#include "simpleMath.h"

using std::ostream;  using std::istream;  using std::endl; using std::cout;

/* CONSTRUCTORS
********************************/
Vector::Vector(int l, double *data) : rows(1), cols(l)
{
	allocSpaceVec();

	for (int i = 0; i < this->cols; i++)
	{
		vec[0][i] = data[i];
	}
};

Vector::Vector(int l) : rows(1), cols(l)
{
	allocSpaceVec();

	for (int i = 0; i < this->cols; i++)
	{
		vec[0][i] = 0;
	}
};

Vector::Vector(const Vector& v) : rows(v.rows), cols(v.cols)
{
	allocSpaceVec();

	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			vec[i][j] = v.vec[i][j];
		}
	}

}

Vector::Vector(const Vector* v) : rows(v->rows), cols(v->cols)
{
	allocSpaceVec();

	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			vec[i][j] = v->vec[i][j];
		}
	}

}

Vector::Vector() : rows(1), cols(1)
{
	allocSpaceVec();
	vec[0][0] = 0;
}

Vector::~Vector()
{
	for (int i = 0; i < rows; ++i) {
		delete[] vec[i];
	}
	delete[] vec;
}

/* GET FUNCTIONS
********************************/
bool	Vector::isRow()
{
	if (this->rows == 1)
		return true;
	else
		return false;
}

bool	Vector::isCol()
{
	if (this->cols == 1)
		return true;
	else
		return false;
}

/* SET FUNCTIONS
********************************/
int		Vector::setLength(int l)
{
	deleteSpaceVec();

	this->rows = 1;
	this->cols = l;

	allocSpaceVec();

	for (int i = 0; i < cols; ++i) {
		vec[0][i] = 0;
	}

	return 0;
}


/* STANDARD MATRIX
********************************/
int		Vector::Zeros()
{
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			vec[i][j] = 0;
		}
	}

	return 0;
}

int		Vector::Ones()
{
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			vec[i][j] = 1;
		}
	}

	return 0;
}


/* MATH OPERATIONS MATRIX
********************************/
int		Vector::Transpose()
{
	Vector tmp = *this;

	this->rows = tmp.cols;
	this->cols = tmp.rows;

	this->allocSpaceVec();
	this->Zeros();

	for (int i = 0; i < tmp.rows; i++)
	{
		for (int j = 0; j < tmp.cols; j++)
		{
			this->vec[j][i] = tmp.vec[i][j];
		}
	}

	return 0;
}

int		Vector::Transpose(Vector *outVec)
{
	outVec->setLength(this->getLength());
	if (this->rows == 1)
		outVec->Transpose();

	for (int i = 0; i < this->rows; i++)
	{
		for (int j = 0; j < this->cols; j++)
		{
			outVec->vec[j][i] = this->vec[i][j];
		}
	}

	return 0;
}

int		Vector::Add(Vector *outVec, Vector *v)
{
	if (this->rows != v->rows || this->cols != v->cols)
		return -1;

	outVec->setLength(this->getLength());
	if (this->cols == 1)
		outVec->Transpose();


	for (int i = 0; i < outVec->rows; i++)
	{
		for (int j = 0; j < outVec->cols; j++)
		{
			outVec->vec[i][j] = this->vec[i][j] + v->vec[i][j];
		}
	}

	return 0;
}

int		Vector::Add(Vector *v)
{
	if (this->rows != v->rows || this->cols != v->cols)
		return -1;

	for (int i = 0; i < this->rows; i++)
	{
		for (int j = 0; j < this->cols; j++)
		{
			this->vec[i][j] = this->vec[i][j] + v->vec[i][j];
		}
	}

	return 0;
}

int		Vector::Subs(Vector *outVec, Vector *v)
{
	if (this->rows != v->rows || this->cols != v->cols)
		return -1;

	outVec->setLength(this->getLength());
	if (this->cols == 1)
		outVec->Transpose();


	for (int i = 0; i < outVec->rows; i++)
	{
		for (int j = 0; j < outVec->cols; j++)
		{
			outVec->vec[i][j] = this->vec[i][j] - v->vec[i][j];
		}
	}

	return 0;
}

int		Vector::Subs(Vector *v)
{
	if (this->rows != v->rows || this->cols != v->cols)
		return -1;

	for (int i = 0; i < this->rows; i++)
	{
		for (int j = 0; j < this->cols; j++)
		{
			this->vec[i][j] = this->vec[i][j] - v->vec[i][j];
		}
	}

	return 0;
}

int		Vector::Mult(Vector *v)
{
	if (this->cols != v->rows)
		return -1;

	if (this->rows != 1 && v->cols != 1)
		return -2;		// The output is a matrix

	Vector tmp = *this;

	this->setLength(1);

	for (int k = 0; k < this->cols; ++k)
		(*this)(0) += tmp.vec[0][k] * v->vec[k][0];

	return 0;
}

int		Vector::Mult(Vector *vout, Vector *v)
{
	if (this->cols != v->rows)
		return -1;

	if (this->rows != 1 && v->cols != 1)
		return -2;		// The output is a matrix

	vout->setLength(1);

	for (int k = 0; k < this->cols; ++k)
		(*vout)(0) += this->vec[0][k] * v->vec[k][0];

	return 0;
}

int		Vector::Mult(Matrix *mout, Vector *v)
{
	if (this->cols != v->rows)
		return -1;

	mout->setDimension(this->rows, v->cols);

	for (int i = 0; i < this->rows; i++)
	{
		for (int j = 0; j < v->cols; j++)
		{
			for (int k = 0; k < this->cols; ++k)
				(*mout)(i, j) += this->vec[i][k] * v->vec[k][j];
		}
	}

	return 0;
}

int		Vector::Mult(Vector *vout, Matrix *m)
{
	if (this->cols != m->getRows())
		return -1;

	vout->setLength(m->getCols());

	for (int j = 0; j < m->getCols(); j++)
	{
		for (int k = 0; k < this->cols; ++k)
			(*vout)(j) += (*this)(k) * (*m)(k, j);
	}

	return 0;
}

int		Vector::Mult(Matrix *m)
{
	if (this->cols != m->getRows())
		return -1;

	Vector tmp = *this;

	this->setLength(m->getCols());

	for (int j = 0; j < m->getCols(); j++)
	{
		for (int k = 0; k < this->cols; ++k)
			(*this)(j) += tmp(k) * (*m)(k, j);
	}

	return 0;
}

int		Vector::Mult(Matrix *mout, Matrix *m)
{
	if (this->cols != m->getRows())
		return -1;

	mout->setDimension(this->rows,m->getCols());
	for (int i = 0; i < mout->getRows(); ++i)
	{
		for (int j = 0; j < mout->getCols(); j++)
		{
			for (int k = 0; k < this->cols; ++k)
				(*mout)(i,j) += this->vec[i][k] * (*m)(k, j);
		}
	}
	
	return 0;
}

int		Vector::ElementMult(Vector* outVec, Vector* v)
{
	if (this->rows != v->rows || this->cols != v->cols)
		return -1;

	outVec->setLength(this->getLength());
	if (this->cols == 1)
		outVec->Transpose();

	for (int i = 0; i < this->rows; ++i) {
		for (int j = 0; j < this->cols; ++j) {
			outVec->vec[i][j] = this->vec[i][j] * v->vec[i][j];
		}
	}

	return 0;

}

int		Vector::ElementMult(Vector* v)
{
	if (this->rows != v->rows || this->cols != v->cols)
		return -1;

	for (int i = 0; i < this->rows; ++i) {
		for (int j = 0; j < this->cols; ++j) {
			this->vec[i][j] = this->vec[i][j] * v->vec[i][j];
		}
	}

	return 0;
}

int		Vector::Mult(Vector* outVec, double n)
{

	outVec->setLength(this->getLength());
	if (this->cols == 1)
		outVec->Transpose();

	for (int i = 0; i < this->rows; ++i) {
		for (int j = 0; j < this->cols; ++j) {
			outVec->vec[i][j] = this->vec[i][j] * n;
		}
	}

	return 0;
}

int		Vector::Mult(double n)
{
	for (int i = 0; i < this->rows; ++i) {
		for (int j = 0; j < this->cols; ++j) {
			this->vec[i][j] *= n;
		}
	}

	return 0;
}

int		Vector::Division(Vector* outVec, double n)
{

	outVec->setLength(this->getLength());
	if (this->cols == 1)
		outVec->Transpose();

	for (int i = 0; i < this->rows; ++i) {
		for (int j = 0; j < this->cols; ++j) {
			outVec->vec[i][j] = this->vec[i][j] / n;
		}
	}

	return 0;
}

int		Vector::Division(double n)
{
	for (int i = 0; i < this->rows; ++i) {
		for (int j = 0; j < this->cols; ++j) {
			this->vec[i][j] /= n;
		}
	}

	return 0;
}

int		Vector::ElementExp(Vector* outVec, int n)
{
	outVec->setLength(this->getLength());
	if (this->cols == 1)
		outVec->Transpose();

	outVec->Ones();

	for (int i = 0; i < this->rows; ++i) {
		for (int j = 0; j < this->cols; ++j) {
			for (int k = 0; k < n; ++k)
			{
				outVec->vec[i][j] *= this->vec[i][j];
			}
		}
	}

	return 0;
}

int		Vector::ElementExp(int n)
{
	if (n == 0)
	{
		this->Ones();
		return 0;
	}

	for (int i = 0; i < this->rows; ++i) {
		for (int j = 0; j < this->cols; ++j) {
			for (int k = 0; k < n - 1; ++k)
			{
				this->vec[i][j] *= this->vec[i][j];
			}
		}
	}

	return 0;
}


/* MATH OPERATIONS VECTORS
********************************/
int		Vector::CrossProd(Vector *outVec, Vector *v)
{
	bool b_row = false;

	if (this->getLength() != 3 || v->getLength() != 3)
		return -1;

	if (this->rows != this->rows)
		return -2;

	if (this->rows == 1)
		b_row = true;

	if (b_row)
	{
		outVec->setLength(3);

		outVec->vec[0][0] = this->vec[0][1] * v->vec[0][2] - this->vec[0][2] * v->vec[0][1];
		outVec->vec[0][1] = this->vec[0][2] * v->vec[0][0] - this->vec[0][0] * v->vec[0][2];
		outVec->vec[0][2] = this->vec[0][0] * v->vec[0][1] - this->vec[0][1] * v->vec[0][0];
	}
	else
	{
		outVec->setLength(3);
		outVec->Transpose();

		outVec->vec[0][0] = this->vec[1][0] * v->vec[2][0] - this->vec[2][0] * v->vec[1][0];
		outVec->vec[1][0] = this->vec[2][0] * v->vec[0][0] - this->vec[0][0] * v->vec[2][0];
		outVec->vec[2][0] = this->vec[0][0] * v->vec[1][0] - this->vec[1][0] * v->vec[0][0];
	}

	return 0;
}

int		Vector::CrossProd(Vector *v)
{
	bool b_row = false;

	if (this->getLength() != 3 || v->getLength() != 3)
		return -1;

	if (this->rows != this->rows)
		return -2;

	if (this->rows == 1)
		b_row = true;

	Vector tmp = *this;

	if (b_row)
	{
		this->vec[0][0] = tmp.vec[0][1] * v->vec[0][2] - tmp.vec[0][2] * v->vec[0][1];
		this->vec[0][1] = tmp.vec[0][2] * v->vec[0][0] - tmp.vec[0][0] * v->vec[0][2];
		this->vec[0][2] = tmp.vec[0][0] * v->vec[0][1] - tmp.vec[0][1] * v->vec[0][0];
	}
	else
	{
		this->vec[0][0] = tmp.vec[1][0] * v->vec[2][0] - tmp.vec[2][0] * v->vec[1][0];
		this->vec[1][0] = tmp.vec[2][0] * v->vec[0][0] - tmp.vec[0][0] * v->vec[2][0];
		this->vec[2][0] = tmp.vec[0][0] * v->vec[1][0] - tmp.vec[1][0] * v->vec[0][0];
	}

	return 0;
}

double	Vector::Norm()
{
	double norm = 0;

	for (int i = 0; i < this->rows; ++i)
	{
		for (int j = 0; j < this->cols; ++j)
		{
			norm += this->vec[i][j] * this->vec[i][j];
		}
	}

	return sqrt(norm);

}

int		Vector::VecNorm(Vector *outVec)
{
	outVec->setLength(this->getLength());

	if (this->cols == 1)
		outVec->Transpose();

	double norm = this->Norm();

	for (int i = 0; i < this->rows; ++i)
	{
		for (int j = 0; j < this->cols; ++j)
		{
			outVec->vec[i][j] = this->vec[i][j] / norm;
		}
	}

	return 0;
}

int		Vector::VecNorm()
{
	double norm = this->Norm();

	for (int i = 0; i < this->rows; ++i)
	{
		for (int j = 0; j < this->cols; ++j)
		{
			this->vec[i][j] = this->vec[i][j] / norm;
		}
	}

	return 0;
}

double	Vector::DotProd(Vector* v)
{
	if (this->getLength() != v->getLength())
		return INFINITY;

	double out = 0;

	for (int i = 0; i < this->getLength(); ++i)
		out += (*this)(i) * (*v)(i);

	return out;
}

/* CONVERSIONS
********************************/
int		Vector::vec2mat(Matrix *m)
{
	m->setDimension(this->rows, this->cols);

	for (int i = 0; i < this->rows; ++i) {
		for (int j = 0; j < this->cols; ++j) {
			(*m)(i,j) = this->vec[i][j];
		}
	}

	return 0;

}

int		Vector::mat2vec(Matrix *m)
{
	if (m->getRows() != 1 && m->getCols() != 1)
		return -1;

	if (m->getRows() == 1)
		this->setLength(m->getCols());
	else
	{
		this->setLength(m->getRows());
		this->Transpose();
	}

	for (int i = 0; i < this->rows; ++i) {
		for (int j = 0; j < this->cols; ++j) {
			this->vec[i][j] = (*m)(i, j);
		}
	}

	return 0;
}

/* PRIVATE HELPER FUNCTIONS
********************************/
void	Vector::allocSpaceVec()
{
	vec = new double*[rows];
	for (int i = 0; i < rows; ++i) {
		vec[i] = new double[cols];
	}
}

void	Vector::deleteSpaceVec()
{
	for (int i = 0; i < rows; ++i) {
		delete[] vec[i];
	}
	delete[] vec;
}

int		Vector::invSqrt(float *out, float x)
{
	long i = 0x5F1F1412 - (*(long*)&x >> 1);
	float tmp = *(float*)&i;
	tmp = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);

	memcpy(out, &tmp, sizeof *out);
	return 0;
}


/* OPERATORS FUNCTIONS
********************************/
ostream& operator<<(ostream& os, const Vector& v)
{
	for (int i = 0; i < v.rows; ++i) {
		os << v.vec[i][0];
		for (int j = 1; j < v.cols; ++j) {
			os << "\t" << v.vec[i][j];
		}
		os << endl;
	}
	return os;
}

ostream& operator<<(ostream& os, const Vector *v)
{
	for (int i = 0; i < v->rows; ++i) {
		os << v->vec[i][0];
		for (int j = 1; j < v->cols; ++j) {
			os << "\t" << v->vec[i][j];
		}
		os << endl;
	}
	return os;
}

istream& operator>>(istream& is, Vector& v)
{
	for (int i = 0; i < v.rows; ++i) {
		for (int j = 0; j < v.cols; ++j) {
			is >> v.vec[i][j];
		}
	}
	return is;
}

Vector& Vector::operator=(const Vector& v)
{
	if (rows != v.rows || cols != v.cols) {

		deleteSpaceVec();

		rows = v.rows;
		cols = v.cols;

		allocSpaceVec();
	}

	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			vec[i][j] = v.vec[i][j];
		}
	}
	return *this;
}