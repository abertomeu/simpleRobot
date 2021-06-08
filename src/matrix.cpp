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
Matrix::Matrix(int r, int c, double *data) : rows(r), cols(c)
{
	allocSpaceMat();
	
	int counter = 0;

	for (int i = 0; i < this->rows; i++)
	{
		for (int j = 0; j < this->cols; j++)
		{
			mat[i][j] = data[counter];
			counter++;
		}
	}
};

Matrix::Matrix(int r, int c) : rows(r), cols(c)
{
    allocSpaceMat();
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            mat[i][j] = 0;
        }
    }
}

Matrix::Matrix(int r, double *data) : rows(r), cols(r)
{
	allocSpaceMat();

	int counter = 0;

	for (int i = 0; i < this->rows; i++)
	{
		for (int j = 0; j < this->cols; j++)
		{
			mat[i][j] = data[counter];
			counter++;
		}
	}
};

Matrix::Matrix(int r) : rows(r), cols(r)
{
	allocSpaceMat();
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			mat[i][j] = 0;
		}
	}
};

Matrix::Matrix(const Matrix& m) : rows(m.rows), cols(m.cols)
{
	allocSpaceMat();
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			mat[i][j] = m.mat[i][j];
		}
	}
}

Matrix::Matrix(const Matrix* m) : rows(m->rows), cols(m->cols)
{
	allocSpaceMat();
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			mat[i][j] = m->mat[i][j];
		}
	}
}

Matrix::Matrix() : rows(1), cols(1)
{
    allocSpaceMat();
    mat[0][0] = 0;
}

Matrix::~Matrix()
{
    for (int i = 0; i < rows; ++i) {
        delete[] mat[i];
    }
    delete[] mat;
}


/* STANDARD MATRIX
********************************/
int		Matrix::Zeros()
{
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			mat[i][j] = 0;
		}
	}

	return 0;
}

int		Matrix::Identity()
{
	if (rows != cols)
		return -1;

	for (int i = 0; i < this->rows; i++)
	{
		for (int j = 0; j < this->cols; j++)
		{
			if (i == j)
				this->mat[j][i] = 1;
			else
				this->mat[j][i] = 0;
		}
	}

	return 0;
}

int		Matrix::Ones()
{
	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			mat[i][j] = 1;
		}
	}

	return 0;
}

/* SET FUNCTIONS
********************************/
int		Matrix::setDimension(int r, int c)
{
	deleteSpaceMat();

	this->rows = r;
	this->cols = c;

	allocSpaceMat();

	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			mat[i][j] = 0;
		}
	}

	return 0;
}

int		Matrix::setDimension(int r)
{
	deleteSpaceMat();

	this->rows = r;
	this->cols = r;

	allocSpaceMat();

	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			mat[i][j] = 0;
		}
	}

	return 0;
}


/* MATH OPERATIONS MATRIX
********************************/
int		Matrix::Transpose()
{
	Matrix tmp = *this;

	this->rows = tmp.cols;
	this->cols = tmp.rows;

	this->allocSpaceMat();
	this->Zeros();

	for (int i = 0; i < tmp.rows; i++)
	{
		for (int j = 0; j < tmp.cols; j++)
		{
			this->mat[j][i] = tmp.mat[i][j];
		}
	}

	return 0;
}

int		Matrix::Transpose(Matrix *outMat)
{
	outMat->setDimension(this->cols, this->rows);

	for (int i = 0; i < this->rows; i++)
	{
		for (int j = 0; j < this->cols; j++)
		{
			outMat->mat[j][i] = this->mat[i][j];
		}
	}

	return 0;
}

double	Matrix::Trace()
{
	if (this->rows != this->cols)
		return -1;

	double tmp = 0;

	for (int i = 0; i < this->rows; ++i) {
		tmp += this->mat[i][i];
	}

	return tmp;
}

int		Matrix::Add(Matrix *outMat, Matrix *m)
{
	if (this->rows != m->rows || this->cols != m->cols)
		return -1;

	outMat->setDimension(this->rows, this->cols);

	for (int i = 0; i < outMat->rows; i++)
	{
		for (int j = 0; j < outMat->cols; j++)
		{
			outMat->mat[i][j] = this->mat[i][j] + m->mat[i][j];
		}
	}

	return 0;
}

int		Matrix::Add(Matrix *m)
{
	if (this->rows != m->rows || this->cols != m->cols)
		return -1;

	for (int i = 0; i < this->rows; i++)
	{
		for (int j = 0; j < this->cols; j++)
		{
			this->mat[i][j] = this->mat[i][j] + m->mat[i][j];
		}
	}

	return 0;
}

int		Matrix::Subs(Matrix *outMat, Matrix *m)
{
	if (this->rows != m->rows || this->cols != m->cols)
		return -1;

	outMat->setDimension(this->rows, this->cols);

	for (int i = 0; i < outMat->rows; i++)
	{
		for (int j = 0; j < outMat->cols; j++)
		{
			outMat->mat[i][j] = this->mat[i][j] - m->mat[i][j];
		}
	}

	return 0;
}

int		Matrix::Subs(Matrix *m)
{
	if (this->rows != m->rows || this->cols != m->cols)
		return -1;

	for (int i = 0; i < this->rows; i++)
	{
		for (int j = 0; j < this->cols; j++)
		{
			this->mat[i][j] = this->mat[i][j] - m->mat[i][j];
		}
	}

	return 0;
}

int		Matrix::Mult(Matrix *outMat, Matrix *m)
{
	if (this->cols != m->rows)
		return -1;

	outMat->setDimension(this->rows, m->cols);

	for (int i = 0; i < outMat->rows; i++)
	{
		for (int j = 0; j < outMat->cols; j++)
		{
			for (int k = 0; k < this->cols; ++k)
				outMat->mat[i][j] += this->mat[i][k] * m->mat[k][j];
		}
	}

	return 0;
}

int		Matrix::Mult(Matrix *m)
{
	if (this->cols != m->rows)
		return -1;

	Matrix *tmp = new Matrix(this);

	this->setDimension(this->rows, m->cols);

	for (int i = 0; i < this->rows; i++)
	{
		for (int j = 0; j < this->cols; j++)
		{
			for (int k = 0; k < tmp->cols; ++k)
				this->mat[i][j] += tmp->mat[i][k] * m->mat[k][j];
		}
	}

	return 0;
}

int		Matrix::Mult(Matrix* outMat, double n)
{

	outMat->setDimension(this->rows, this->cols);

	for (int i = 0; i < this->rows; ++i) {
		for (int j = 0; j < this->cols; ++j) {
			outMat->mat[i][j] = this->mat[i][j] * n;
		}
	}

	return 0;
}

int		Matrix::Mult(double n)
{
	for (int i = 0; i < this->rows; ++i) {
		for (int j = 0; j < this->cols; ++j) {
			this->mat[i][j] *= n;
		}
	}

	return 0;
}

int		Matrix::Division(Matrix* outMat, double n)
{

	outMat->setDimension(this->rows, this->cols);

	for (int i = 0; i < this->rows; ++i) {
		for (int j = 0; j < this->cols; ++j) {
			outMat->mat[i][j] = this->mat[i][j] / n;
		}
	}

	return 0;
}

int		Matrix::Division(double n)
{
	for (int i = 0; i < this->rows; ++i) {
		for (int j = 0; j < this->cols; ++j) {
			this->mat[i][j] /= n;
		}
	}

	return 0;
}

int		Matrix::Inv3x3()
{
	if (this->cols != this->rows)
		return -1;

	if (this->rows != 3)
		return -2;

	double a, b, c, d, e, f, g, h, i;
	a = this->mat[0][0];
	b = this->mat[0][1];
	c = this->mat[0][2];
	d = this->mat[1][0];
	e = this->mat[1][1];
	f = this->mat[1][2];
	g = this->mat[2][0];
	h = this->mat[2][1];
	i = this->mat[2][2];

	double deter = a * e * i + b * f * g + c * d * h - c * e * g - b * d * i - a * f * h;

	if (deter == 0)
		return -3;

	this->mat[0][0] = (1 / deter) * (e * i - f * h);
	this->mat[0][1] = (1 / deter) * (c * h - b * i);
	this->mat[0][2] = (1 / deter) * (b * f - c * e);
	this->mat[1][0] = (1 / deter) * (f * g - d * i);
	this->mat[1][1] = (1 / deter) * (a * i - c * g);
	this->mat[1][2] = (1 / deter) * (c * d - a * f);
	this->mat[2][0] = (1 / deter) * (d * h - e * g);
	this->mat[2][1] = (1 / deter) * (g * b - a * h);
	this->mat[2][2] = (1 / deter) * (a * e - b * d);

	return 0;
}

int		Matrix::Inv3x3(Matrix* outMat)
{
	if (this->cols != this->rows)
		return -1;

	if (this->rows != 3)
		return -2;

	outMat->setDimension(this->rows, this->cols);

	double a, b, c, d, e, f, g, h, i;
	a = this->mat[0][0];
	b = this->mat[0][1];
	c = this->mat[0][2];
	d = this->mat[1][0];
	e = this->mat[1][1];
	f = this->mat[1][2];
	g = this->mat[2][0];
	h = this->mat[2][1];
	i = this->mat[2][2];

	double deter = a * e * i + b * f * g + c * d * h - c * e * g - b * d * i - a * f * h;
	
	if (deter == 0)
		return -3;

	outMat->mat[0][0] = (1 / deter) * (e * i - f * h);
	outMat->mat[0][1] = (1 / deter) * (c * h - b * i);
	outMat->mat[0][2] = (1 / deter) * (b * f - c * e);
	outMat->mat[1][0] = (1 / deter) * (f * g - d * i);
	outMat->mat[1][1] = (1 / deter) * (a * i - c * g);
	outMat->mat[1][2] = (1 / deter) * (c * d - a * f);
	outMat->mat[2][0] = (1 / deter) * (d * h - e * g);
	outMat->mat[2][1] = (1 / deter) * (g * b - a * h);
	outMat->mat[2][2] = (1 / deter) * (a * e - b * d);

	return 0;
}

int		Matrix::Inv4x4()
{
	if (this->cols != this->rows)
		return -1;

	if (this->rows != 4)
		return -2;

	Matrix *tmp = new Matrix(this);

	(*this)(0,0) = (*tmp)(1,1) * (*tmp)(2,2) * (*tmp)(3,3) -
		(*tmp)(1,1) * (*tmp)(2,3) * (*tmp)(3,2) -
		(*tmp)(2,1) * (*tmp)(1,2) * (*tmp)(3,3) +
		(*tmp)(2,1) * (*tmp)(1,3) * (*tmp)(3,2) +
		(*tmp)(3,1) * (*tmp)(1,2) * (*tmp)(2,3) -
		(*tmp)(3,1) * (*tmp)(1,3) * (*tmp)(2,2);

	(*this)(1,0) = -(*tmp)(1,0) * (*tmp)(2,2) * (*tmp)(3,3) +
		(*tmp)(1,0) * (*tmp)(2,3) * (*tmp)(3,2) +
		(*tmp)(2,0) * (*tmp)(1,2) * (*tmp)(3,3) -
		(*tmp)(2,0) * (*tmp)(1,3) * (*tmp)(3,2) -
		(*tmp)(3,0) * (*tmp)(1,2) * (*tmp)(2,3) +
		(*tmp)(3,0) * (*tmp)(1,3) * (*tmp)(2,2);

	(*this)(2,0) = (*tmp)(1,0) * (*tmp)(2,1) * (*tmp)(3,3) -
		(*tmp)(1,0) * (*tmp)(2,3) * (*tmp)(3,1) -
		(*tmp)(2,0) * (*tmp)(1,1) * (*tmp)(3,3) +
		(*tmp)(2,0) * (*tmp)(1,3) * (*tmp)(3,1) +
		(*tmp)(3,0) * (*tmp)(1,1) * (*tmp)(2,3) -
		(*tmp)(3,0) * (*tmp)(1,3) * (*tmp)(2,1);

	(*this)(3,0) = -(*tmp)(1,0) * (*tmp)(2,1) * (*tmp)(3,2) +
		(*tmp)(1,0) * (*tmp)(2,2) * (*tmp)(3,1) +
		(*tmp)(2,0) * (*tmp)(1,1) * (*tmp)(3,2) -
		(*tmp)(2,0) * (*tmp)(1,2) * (*tmp)(3,1) -
		(*tmp)(3,0) * (*tmp)(1,1) * (*tmp)(2,2) +
		(*tmp)(3,0) * (*tmp)(1,2) * (*tmp)(2,1);

	(*this)(0,1) = -(*tmp)(0,1) * (*tmp)(2,2) * (*tmp)(3,3) +
		(*tmp)(0,1) * (*tmp)(2,3) * (*tmp)(3,2) +
		(*tmp)(2,1) * (*tmp)(0,2) * (*tmp)(3,3) -
		(*tmp)(2,1) * (*tmp)(0,3) * (*tmp)(3,2) -
		(*tmp)(3,1) * (*tmp)(0,2) * (*tmp)(2,3) +
		(*tmp)(3,1) * (*tmp)(0,3) * (*tmp)(2,2);

	(*this)(1,1) = (*tmp)(0,0) * (*tmp)(2,2) * (*tmp)(3,3) -
		(*tmp)(0,0) * (*tmp)(2,3) * (*tmp)(3,2) -
		(*tmp)(2,0) * (*tmp)(0,2) * (*tmp)(3,3) +
		(*tmp)(2,0) * (*tmp)(0,3) * (*tmp)(3,2) +
		(*tmp)(3,0) * (*tmp)(0,2) * (*tmp)(2,3) -
		(*tmp)(3,0) * (*tmp)(0,3) * (*tmp)(2,2);

	(*this)(2,1) = -(*tmp)(0,0) * (*tmp)(2,1) * (*tmp)(3,3) +
		(*tmp)(0,0) * (*tmp)(2,3) * (*tmp)(3,1) +
		(*tmp)(2,0) * (*tmp)(0,1) * (*tmp)(3,3) -
		(*tmp)(2,0) * (*tmp)(0,3) * (*tmp)(3,1) -
		(*tmp)(3,0) * (*tmp)(0,1) * (*tmp)(2,3) +
		(*tmp)(3,0) * (*tmp)(0,3) * (*tmp)(2,1);

	(*this)(3,1) = (*tmp)(0,0) * (*tmp)(2,1) * (*tmp)(3,2) -
		(*tmp)(0,0) * (*tmp)(2,2) * (*tmp)(3,1) -
		(*tmp)(2,0) * (*tmp)(0,1) * (*tmp)(3,2) +
		(*tmp)(2,0) * (*tmp)(0,2) * (*tmp)(3,1) +
		(*tmp)(3,0) * (*tmp)(0,1) * (*tmp)(2,2) -
		(*tmp)(3,0) * (*tmp)(0,2) * (*tmp)(2,1);

	(*this)(0,2) = (*tmp)(0,1) * (*tmp)(1,2) * (*tmp)(3,3) -
		(*tmp)(0,1) * (*tmp)(1,3) * (*tmp)(3,2) -
		(*tmp)(1,1) * (*tmp)(0,2) * (*tmp)(3,3) +
		(*tmp)(1,1) * (*tmp)(0,3) * (*tmp)(3,2) +
		(*tmp)(3,1) * (*tmp)(0,2) * (*tmp)(1,3) -
		(*tmp)(3,1) * (*tmp)(0,3) * (*tmp)(1,2);

	(*this)(1,2) = -(*tmp)(0,0) * (*tmp)(1,2) * (*tmp)(3,3) +
		(*tmp)(0,0) * (*tmp)(1,3) * (*tmp)(3,2) +
		(*tmp)(1,0) * (*tmp)(0,2) * (*tmp)(3,3) -
		(*tmp)(1,0) * (*tmp)(0,3) * (*tmp)(3,2) -
		(*tmp)(3,0) * (*tmp)(0,2) * (*tmp)(1,3) +
		(*tmp)(3,0) * (*tmp)(0,3) * (*tmp)(1,2);

	(*this)(2,2) = (*tmp)(0,0) * (*tmp)(1,1) * (*tmp)(3,3) -
		(*tmp)(0,0) * (*tmp)(1,3) * (*tmp)(3,1) -
		(*tmp)(1,0) * (*tmp)(0,1) * (*tmp)(3,3) +
		(*tmp)(1,0) * (*tmp)(0,3) * (*tmp)(3,1) +
		(*tmp)(3,0) * (*tmp)(0,1) * (*tmp)(1,3) -
		(*tmp)(3,0) * (*tmp)(0,3) * (*tmp)(1,1);

	(*this)(3,2) = -(*tmp)(0,0) * (*tmp)(1,1) * (*tmp)(3,2) +
		(*tmp)(0,0) * (*tmp)(1,2) * (*tmp)(3,1) +
		(*tmp)(1,0) * (*tmp)(0,1) * (*tmp)(3,2) -
		(*tmp)(1,0) * (*tmp)(0,2) * (*tmp)(3,1) -
		(*tmp)(3,0) * (*tmp)(0,1) * (*tmp)(1,2) +
		(*tmp)(3,0) * (*tmp)(0,2) * (*tmp)(1,1);

	(*this)(0,3) = -(*tmp)(0,1) * (*tmp)(1,2) * (*tmp)(2,3) +
		(*tmp)(0,1) * (*tmp)(1,3) * (*tmp)(2,2) +
		(*tmp)(1,1) * (*tmp)(0,2) * (*tmp)(2,3) -
		(*tmp)(1,1) * (*tmp)(0,3) * (*tmp)(2,2) -
		(*tmp)(2,1) * (*tmp)(0,2) * (*tmp)(1,3) +
		(*tmp)(2,1) * (*tmp)(0,3) * (*tmp)(1,2);

	(*this)(1,3) = (*tmp)(0,0) * (*tmp)(1,2) * (*tmp)(2,3) -
		(*tmp)(0,0) * (*tmp)(1,3) * (*tmp)(2,2) -
		(*tmp)(1,0) * (*tmp)(0,2) * (*tmp)(2,3) +
		(*tmp)(1,0) * (*tmp)(0,3) * (*tmp)(2,2) +
		(*tmp)(2,0) * (*tmp)(0,2) * (*tmp)(1,3) -
		(*tmp)(2,0) * (*tmp)(0,3) * (*tmp)(1,2);

	(*this)(2,3) = -(*tmp)(0,0) * (*tmp)(1,1) * (*tmp)(2,3) +
		(*tmp)(0,0) * (*tmp)(1,3) * (*tmp)(2,1) +
		(*tmp)(1,0) * (*tmp)(0,1) * (*tmp)(2,3) -
		(*tmp)(1,0) * (*tmp)(0,3) * (*tmp)(2,1) -
		(*tmp)(2,0) * (*tmp)(0,1) * (*tmp)(1,3) +
		(*tmp)(2,0) * (*tmp)(0,3) * (*tmp)(1,1);

	(*this)(3,3) = (*tmp)(0,0) * (*tmp)(1,1) * (*tmp)(2,2) -
		(*tmp)(0,0) * (*tmp)(1,2) * (*tmp)(2,1) -
		(*tmp)(1,0) * (*tmp)(0,1) * (*tmp)(2,2) +
		(*tmp)(1,0) * (*tmp)(0,2) * (*tmp)(2,1) +
		(*tmp)(2,0) * (*tmp)(0,1) * (*tmp)(1,2) -
		(*tmp)(2,0) * (*tmp)(0,2) * (*tmp)(1,1);

	double det = (*tmp)(0,0) * (*this)(0,0) + (*tmp)(0,1) * (*this)(1,0) + (*tmp)(0,2) * (*this)(2,0) + (*tmp)(0,3) * (*this)(3,0);

	if (det == 0)
		return -3;

	for (int i = 0; i < this->rows; ++i) {
		for (int j = 0; j < this->cols; ++j) {
			this->mat[i][j] /= det;
		}
	}

	return 0;
}

int		Matrix::Inv4x4(Matrix* outMat)
{
	if (this->cols != this->rows)
		return -1;

	if (this->rows != 4)
		return -2;

	outMat->setDimension(this->rows, this->cols);

	(*outMat)(0, 0) = (*this)(1, 1) * (*this)(2, 2) * (*this)(3, 3) -
		(*this)(1, 1) * (*this)(2, 3) * (*this)(3, 2) -
		(*this)(2, 1) * (*this)(1, 2) * (*this)(3, 3) +
		(*this)(2, 1) * (*this)(1, 3) * (*this)(3, 2) +
		(*this)(3, 1) * (*this)(1, 2) * (*this)(2, 3) -
		(*this)(3, 1) * (*this)(1, 3) * (*this)(2, 2);

	(*outMat)(1, 0) = -(*this)(1, 0) * (*this)(2, 2) * (*this)(3, 3) +
		(*this)(1, 0) * (*this)(2, 3) * (*this)(3, 2) +
		(*this)(2, 0) * (*this)(1, 2) * (*this)(3, 3) -
		(*this)(2, 0) * (*this)(1, 3) * (*this)(3, 2) -
		(*this)(3, 0) * (*this)(1, 2) * (*this)(2, 3) +
		(*this)(3, 0) * (*this)(1, 3) * (*this)(2, 2);

	(*outMat)(2, 0) = (*this)(1, 0) * (*this)(2, 1) * (*this)(3, 3) -
		(*this)(1, 0) * (*this)(2, 3) * (*this)(3, 1) -
		(*this)(2, 0) * (*this)(1, 1) * (*this)(3, 3) +
		(*this)(2, 0) * (*this)(1, 3) * (*this)(3, 1) +
		(*this)(3, 0) * (*this)(1, 1) * (*this)(2, 3) -
		(*this)(3, 0) * (*this)(1, 3) * (*this)(2, 1);

	(*outMat)(3, 0) = -(*this)(1, 0) * (*this)(2, 1) * (*this)(3, 2) +
		(*this)(1, 0) * (*this)(2, 2) * (*this)(3, 1) +
		(*this)(2, 0) * (*this)(1, 1) * (*this)(3, 2) -
		(*this)(2, 0) * (*this)(1, 2) * (*this)(3, 1) -
		(*this)(3, 0) * (*this)(1, 1) * (*this)(2, 2) +
		(*this)(3, 0) * (*this)(1, 2) * (*this)(2, 1);

	(*outMat)(0, 1) = -(*this)(0, 1) * (*this)(2, 2) * (*this)(3, 3) +
		(*this)(0, 1) * (*this)(2, 3) * (*this)(3, 2) +
		(*this)(2, 1) * (*this)(0, 2) * (*this)(3, 3) -
		(*this)(2, 1) * (*this)(0, 3) * (*this)(3, 2) -
		(*this)(3, 1) * (*this)(0, 2) * (*this)(2, 3) +
		(*this)(3, 1) * (*this)(0, 3) * (*this)(2, 2);

	(*outMat)(1, 1) = (*this)(0, 0) * (*this)(2, 2) * (*this)(3, 3) -
		(*this)(0, 0) * (*this)(2, 3) * (*this)(3, 2) -
		(*this)(2, 0) * (*this)(0, 2) * (*this)(3, 3) +
		(*this)(2, 0) * (*this)(0, 3) * (*this)(3, 2) +
		(*this)(3, 0) * (*this)(0, 2) * (*this)(2, 3) -
		(*this)(3, 0) * (*this)(0, 3) * (*this)(2, 2);

	(*outMat)(2, 1) = -(*this)(0, 0) * (*this)(2, 1) * (*this)(3, 3) +
		(*this)(0, 0) * (*this)(2, 3) * (*this)(3, 1) +
		(*this)(2, 0) * (*this)(0, 1) * (*this)(3, 3) -
		(*this)(2, 0) * (*this)(0, 3) * (*this)(3, 1) -
		(*this)(3, 0) * (*this)(0, 1) * (*this)(2, 3) +
		(*this)(3, 0) * (*this)(0, 3) * (*this)(2, 1);

	(*outMat)(3, 1) = (*this)(0, 0) * (*this)(2, 1) * (*this)(3, 2) -
		(*this)(0, 0) * (*this)(2, 2) * (*this)(3, 1) -
		(*this)(2, 0) * (*this)(0, 1) * (*this)(3, 2) +
		(*this)(2, 0) * (*this)(0, 2) * (*this)(3, 1) +
		(*this)(3, 0) * (*this)(0, 1) * (*this)(2, 2) -
		(*this)(3, 0) * (*this)(0, 2) * (*this)(2, 1);

	(*outMat)(0, 2) = (*this)(0, 1) * (*this)(1, 2) * (*this)(3, 3) -
		(*this)(0, 1) * (*this)(1, 3) * (*this)(3, 2) -
		(*this)(1, 1) * (*this)(0, 2) * (*this)(3, 3) +
		(*this)(1, 1) * (*this)(0, 3) * (*this)(3, 2) +
		(*this)(3, 1) * (*this)(0, 2) * (*this)(1, 3) -
		(*this)(3, 1) * (*this)(0, 3) * (*this)(1, 2);

	(*outMat)(1, 2) = -(*this)(0, 0) * (*this)(1, 2) * (*this)(3, 3) +
		(*this)(0, 0) * (*this)(1, 3) * (*this)(3, 2) +
		(*this)(1, 0) * (*this)(0, 2) * (*this)(3, 3) -
		(*this)(1, 0) * (*this)(0, 3) * (*this)(3, 2) -
		(*this)(3, 0) * (*this)(0, 2) * (*this)(1, 3) +
		(*this)(3, 0) * (*this)(0, 3) * (*this)(1, 2);

	(*outMat)(2, 2) = (*this)(0, 0) * (*this)(1, 1) * (*this)(3, 3) -
		(*this)(0, 0) * (*this)(1, 3) * (*this)(3, 1) -
		(*this)(1, 0) * (*this)(0, 1) * (*this)(3, 3) +
		(*this)(1, 0) * (*this)(0, 3) * (*this)(3, 1) +
		(*this)(3, 0) * (*this)(0, 1) * (*this)(1, 3) -
		(*this)(3, 0) * (*this)(0, 3) * (*this)(1, 1);

	(*outMat)(3, 2) = -(*this)(0, 0) * (*this)(1, 1) * (*this)(3, 2) +
		(*this)(0, 0) * (*this)(1, 2) * (*this)(3, 1) +
		(*this)(1, 0) * (*this)(0, 1) * (*this)(3, 2) -
		(*this)(1, 0) * (*this)(0, 2) * (*this)(3, 1) -
		(*this)(3, 0) * (*this)(0, 1) * (*this)(1, 2) +
		(*this)(3, 0) * (*this)(0, 2) * (*this)(1, 1);

	(*outMat)(0, 3) = -(*this)(0, 1) * (*this)(1, 2) * (*this)(2, 3) +
		(*this)(0, 1) * (*this)(1, 3) * (*this)(2, 2) +
		(*this)(1, 1) * (*this)(0, 2) * (*this)(2, 3) -
		(*this)(1, 1) * (*this)(0, 3) * (*this)(2, 2) -
		(*this)(2, 1) * (*this)(0, 2) * (*this)(1, 3) +
		(*this)(2, 1) * (*this)(0, 3) * (*this)(1, 2);

	(*outMat)(1, 3) = (*this)(0, 0) * (*this)(1, 2) * (*this)(2, 3) -
		(*this)(0, 0) * (*this)(1, 3) * (*this)(2, 2) -
		(*this)(1, 0) * (*this)(0, 2) * (*this)(2, 3) +
		(*this)(1, 0) * (*this)(0, 3) * (*this)(2, 2) +
		(*this)(2, 0) * (*this)(0, 2) * (*this)(1, 3) -
		(*this)(2, 0) * (*this)(0, 3) * (*this)(1, 2);

	(*outMat)(2, 3) = -(*this)(0, 0) * (*this)(1, 1) * (*this)(2, 3) +
		(*this)(0, 0) * (*this)(1, 3) * (*this)(2, 1) +
		(*this)(1, 0) * (*this)(0, 1) * (*this)(2, 3) -
		(*this)(1, 0) * (*this)(0, 3) * (*this)(2, 1) -
		(*this)(2, 0) * (*this)(0, 1) * (*this)(1, 3) +
		(*this)(2, 0) * (*this)(0, 3) * (*this)(1, 1);

	(*outMat)(3, 3) = (*this)(0, 0) * (*this)(1, 1) * (*this)(2, 2) -
		(*this)(0, 0) * (*this)(1, 2) * (*this)(2, 1) -
		(*this)(1, 0) * (*this)(0, 1) * (*this)(2, 2) +
		(*this)(1, 0) * (*this)(0, 2) * (*this)(2, 1) +
		(*this)(2, 0) * (*this)(0, 1) * (*this)(1, 2) -
		(*this)(2, 0) * (*this)(0, 2) * (*this)(1, 1);

	double det = (*this)(0, 0) * (*outMat)(0, 0) + (*this)(0, 1) * (*outMat)(1, 0) + (*this)(0, 2) * (*outMat)(2, 0) + (*this)(0, 3) * (*outMat)(3, 0);

	if (det == 0)
		return -3;

	for (int i = 0; i < this->rows; ++i) {
		for (int j = 0; j < this->cols; ++j) {
			(*outMat)(i,j) /= det;
		}
	}

	return 0;
}

int		Matrix::Inverse()
{

	if (this->rows != this->cols)
		return -1;

	Matrix Id(this->rows);
	Id.Identity();
	Matrix *tmp = new Matrix(this);

	tmp->augment(&Id);
	tmp->gaussianEliminate();
	tmp->rowReduceFromGaussian();
	
	for (int i = 0; i < this->rows; ++i) 
	{
		for (int j = 0; j < this->cols; ++j) 
		{
			(*this)(i, j) = (*tmp)(i, j + cols);
		}
	}

	return 0;
}

int		Matrix::Inverse(Matrix* outMat)
{
	*outMat = *this;

	Matrix Id(outMat->rows);
	Id.Identity();
	Matrix tmp = *outMat;

	tmp.augment(&Id);
	tmp.gaussianEliminate();
	tmp.rowReduceFromGaussian();

	for (int i = 0; i < outMat->rows; ++i)
	{
		for (int j = 0; j < outMat->cols; ++j)
		{
			(*outMat)(i, j) = tmp(i, j + cols);
		}
	}

	return 0;
}


/* HELPER FUNCTIONS FOR INVERSE
********************************/
int		Matrix::augment(Matrix *B)
{
	if (this->rows != B->rows)
		return -1;

	Matrix *A = new Matrix(this);
	this->setDimension(A->rows, A->cols + B->cols);

	for (int i = 0; i < this->rows; ++i) {
		for (int j = 0; j < this->cols; ++j) {
			if (j < A->cols)
				(*this)(i, j) = (*A)(i, j);
			else
				(*this)(i, j) = (*B)(i, j - B->cols);
		}
	}
	return 0;
}

int		Matrix::augment(Matrix* AB, Matrix *B)
{
	AB->setDimension(this->rows, this->cols + B->cols);

	for (int i = 0; i < AB->rows; ++i) {
		for (int j = 0; j < AB->cols; ++j) {
			if (j < this->cols)
				(*AB)(i, j) = (*this)(i, j);
			else
				(*AB)(i, j) = (*B)(i, j - B->cols);
		}
	}
	return 0;
}

int		Matrix::swapRows(int r1, int r2)
{
	double tmp;

	for (int i = 0; i<this->cols; ++i) {
		tmp = this->mat[r1][i];
		this->mat[r1][i] = this->mat[r2][i];
		this->mat[r2][i] = tmp;
	}

	return 0;
}

int		Matrix::gaussianEliminate()
{
	int rows = this->rows;
	int cols = this->cols;
	int Acols = cols - 1;

	double cur_abs;
	int max_row;
	double max_val;
	bool pivot_found;

	int i = 0; // row tracker
	int j = 0; // column tracker

			   // iterate through the rows
	while (i < rows)
	{
		// find a pivot for the row
		pivot_found = false;
		while (j < Acols && !pivot_found)
		{
			if ((*this)(i, j) != 0) { // pivot not equal to 0
				pivot_found = true;
			}
			else { // check for a possible swap
				max_row = i;
				max_val = 0;
				for (int k = i + 1; k < rows; ++k)
				{
					cur_abs = (*this)(k, j) >= 0 ? (*this)(k, j) : -1 * (*this)(k, j);
					if (cur_abs > max_val)
					{
						max_row = k;
						max_val = cur_abs;
					}
				}
				if (max_row != i) {
					this->swapRows(max_row, i);
					pivot_found = true;
				}
				else {
					j++;
				}
			}
		}

		// perform elimination as normal if pivot was found
		if (pivot_found)
		{
			for (int t = i + 1; t < rows; ++t) {
				for (int s = j + 1; s < cols; ++s) {
					(*this)(t, s) = (*this)(t, s) - (*this)(i, s) * ((*this)(t, j) / (*this)(i, j));
					if ((*this)(t, s) < EPS && (*this)(t, s) > -1 * EPS)
						(*this)(t, s) = 0;
				}
				(*this)(t, j) = 0;
			}
		}

		i++;
		j++;
	}

	return 0;
}

int		Matrix::gaussianEliminate(Matrix* outMat)
{
	*outMat = *this;

	int rows = outMat->rows;
	int cols = outMat->cols;
	int Acols = cols - 1;

	double cur_abs;
	int max_row;
	double max_val;
	bool pivot_found;

	int i = 0; // row tracker
	int j = 0; // column tracker

			   // iterate through the rows
	while (i < rows)
	{
		// find a pivot for the row
		pivot_found = false;
		while (j < Acols && !pivot_found)
		{
			if ((*outMat)(i, j) != 0) { // pivot not equal to 0
				pivot_found = true;
			}
			else { // check for a possible swap
				max_row = i;
				max_val = 0;
				for (int k = i + 1; k < rows; ++k)
				{
					cur_abs = (*outMat)(k, j) >= 0 ? (*outMat)(k, j) : -1 * (*outMat)(k, j);
					if (cur_abs > max_val)
					{
						max_row = k;
						max_val = cur_abs;
					}
				}
				if (max_row != i) {
					outMat->swapRows(max_row, i);
					pivot_found = true;
				}
				else {
					j++;
				}
			}
		}

		// perform elimination as normal if pivot was found
		if (pivot_found)
		{
			for (int t = i + 1; t < rows; ++t) {
				for (int s = j + 1; s < cols; ++s) {
					(*outMat)(t, s) = (*outMat)(t, s) - (*outMat)(i, s) * ((*outMat)(t, j) / (*outMat)(i, j));
					if ((*outMat)(t, s) < EPS && (*outMat)(t, s) > -1 * EPS)
						(*outMat)(t, s) = 0;
				}
				(*outMat)(t, j) = 0;
			}
		}

		i++;
		j++;
	}

	return 0;
}

int		Matrix::rowReduceFromGaussian()
{

	int rows = this->rows;
	int cols = this->cols;

	int i = rows - 1; // row tracker
	int j = cols - 2; // column tracker

					  // iterate through every row
	while (i >= 0)
	{
		// find the pivot column
		int k = j - 1;
		while (k >= 0) {
			if ((*this)(i, k) != 0)
				j = k;
			k--;
		}

		// zero out elements above pivots if pivot not 0
		if ((*this)(i, j) != 0) {

			for (int t = i - 1; t >= 0; --t) {
				for (int s = 0; s < cols; ++s) {
					if (s != j) {
						(*this)(t, s) = (*this)(t, s) - (*this)(i, s) * ((*this)(t, j) / (*this)(i, j));
						if ((*this)(t, s) < EPS && (*this)(t, s) > -1 * EPS)
							(*this)(t, s) = 0;
					}
				}
				(*this)(t, j) = 0;
			}

			// divide row by pivot
			for (int k = j + 1; k < cols; ++k) {
				(*this)(i, k) = (*this)(i, k) / (*this)(i, j);
				if ((*this)(i, k) < EPS && (*this)(i, k) > -1 * EPS)
					(*this)(i, k) = 0;
			}
			(*this)(i, j) = 1;

		}

		i--;
		j--;
	}

	return 0;
}

int		Matrix::rowReduceFromGaussian(Matrix* outMat)
{

	*outMat = *this;

	int rows = outMat->rows;
	int cols = outMat->cols;

	int i = rows - 1; // row tracker
	int j = cols - 2; // column tracker

					  // iterate through every row
	while (i >= 0)
	{
		// find the pivot column
		int k = j - 1;
		while (k >= 0) {
			if ((*outMat)(i, k) != 0)
				j = k;
			k--;
		}

		// zero out elements above pivots if pivot not 0
		if ((*outMat)(i, j) != 0) {

			for (int t = i - 1; t >= 0; --t) {
				for (int s = 0; s < cols; ++s) {
					if (s != j) {
						(*outMat)(t, s) = (*outMat)(t, s) - (*outMat)(i, s) * ((*outMat)(t, j) / (*outMat)(i, j));
						if ((*outMat)(t, s) < EPS && (*outMat)(t, s) > -1 * EPS)
							(*outMat)(t, s) = 0;
					}
				}
				(*outMat)(t, j) = 0;
			}

			// divide row by pivot
			for (int k = j + 1; k < cols; ++k) {
				(*outMat)(i, k) = (*outMat)(i, k) / (*outMat)(i, j);
				if ((*outMat)(i, k) < EPS && (*outMat)(i, k) > -1 * EPS)
					(*outMat)(i, k) = 0;
			}
			(*outMat)(i, j) = 1;

		}

		i--;
		j--;
	}

	return 0;
}


/* PRIVATE HELPER FUNCTIONS
 ********************************/
void	Matrix::allocSpaceMat()
{
    mat = new double*[rows];
    for (int i = 0; i < rows; ++i) {
        mat[i] = new double[cols];
    }
}

void	Matrix::deleteSpaceMat()
{
	for (int i = 0; i < rows; ++i) {
		delete[] mat[i];
	}
	delete[] mat;
}

int		Matrix::invSqrt(float *out, float x)
{
	long i = 0x5F1F1412 - (*(long*)&x >> 1);
	float tmp = *(float*)&i;
	tmp = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);

	memcpy(out, &tmp, sizeof *out);
	return 0;
}

/* OPERATORS FUNCTIONS
 ********************************/
ostream& operator<<(ostream& os, const Matrix& m)
{
	for (int i = 0; i < m.rows; ++i) {
		os << m.mat[i][0];
		for (int j = 1; j < m.cols; ++j) {
			os << "\t" << m.mat[i][j];
		}
		os << endl;
	}
	return os;
}

ostream& operator<<(ostream& os, const Matrix *m)
{
	for (int i = 0; i < m->rows; ++i) {
		os << m->mat[i][0];
		for (int j = 1; j < m->cols; ++j) {
			os << "\t" << m->mat[i][j];
		}
		os << endl;
	}
	return os;
}

istream& operator>>(istream& is, Matrix& m)
{
    for (int i = 0; i < m.rows; ++i) {
        for (int j = 0; j < m.cols; ++j) {
            is >> m.mat[i][j];
        }
    }
    return is;
}

Matrix& Matrix::operator=(const Matrix& m)
{
	if (rows != m.rows || cols != m.cols) {
		
		deleteSpaceMat();

		rows = m.rows;
		cols = m.cols;
		
		allocSpaceMat();
	}

	for (int i = 0; i < rows; ++i) {
		for (int j = 0; j < cols; ++j) {
			mat[i][j] = m.mat[i][j];
		}
	}
	return *this;
}
