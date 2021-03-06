// -*- coding: utf-8 -*-
// -*- lsst - c++ - *-

/**
 * @author : % (Arturo Bertomeu-Motos)
 * @email : % (arturobm90@gmail.com)
 * @institution : % (Biomedical Neuroengineering Research Group (UMH) (https://nbio.umh.es/))
 */

#include "simpleRobot.h"

/* CONSTRUCTOR
********************************/
Robot::Robot(int DoFs, string joints, Vector *theta, Vector *d, Vector *a, Vector *alpha)
{
	this->DoFs = DoFs;
	this->Joints = joints;
	this->Theta = new Vector(theta);
	this->D = new Vector(d);
	this->A = new Vector(a);
	this->Alpha = new Vector(alpha);
}

Robot::Robot()
{
	this->DoFs = 0;
	this->Joints = "";
	this->Theta = new Vector;
	this->D = new Vector;
	this->A = new Vector;
	this->Alpha = new Vector;
}


/* GET FUNCTIONS
********************************/
Vector*		Robot::getTheta()
{
	return this->Theta;
}

double		Robot::getTheta(int idx)
{
	if (idx > this->DoFs)
		return INFINITY;

	return (*Theta)(idx);
}

Vector*		Robot::getD()
{
	return this->D;
}

double		Robot::getD(int idx)
{
	if (idx > this->DoFs)
		return INFINITY;

	return (*D)(idx);
}

Vector*		Robot::getA()
{
	return this->A;
}

double		Robot::getA(int idx)
{
	if (idx > this->DoFs)
		return INFINITY;

	return (*A)(idx);
}

Vector*		Robot::getAlpha()
{
	return this->Alpha;
}

double		Robot::getAlpha(int idx)
{
	if (idx > this->DoFs)
		return INFINITY;

	return (*Alpha)(idx);
}


/* GENERIC FUNCTIONS
********************************/
int		Robot::denavit(Matrix *m, double theta, double d, double a, double alpha)
{
	m->setDimension(4, 4);
	(*m)(0, 0) = cos(theta);
	(*m)(0, 1) = -cos(alpha)*sin(theta);
	(*m)(0, 2) = sin(alpha)*sin(theta);
	(*m)(0, 3) = a * cos(theta);
	(*m)(1, 0) = sin(theta);
	(*m)(1, 1) = cos(alpha)*cos(theta);
	(*m)(1, 2) = -sin(alpha)*cos(theta);
	(*m)(1, 3) = a * sin(theta);
	(*m)(2, 0) = 0;
	(*m)(2, 1) = sin(alpha);
	(*m)(2, 2) = cos(alpha);
	(*m)(2, 3) = d;
	(*m)(3, 0) = 0;
	(*m)(3, 1) = 0;
	(*m)(3, 2) = 0;
	(*m)(3, 3) = 1;

	return 0;
}

int		Robot::rotx(Matrix *m, double theta)
{
	double ct = cos(theta);
	double st = sin(theta);

	m->setDimension(3, 3);
	(*m)(0, 0) = 1;
	(*m)(0, 1) = 0;
	(*m)(0, 2) = 0;
	(*m)(1, 0) = 0;
	(*m)(1, 1) = ct;
	(*m)(1, 2) = -st;
	(*m)(2, 0) = 0;
	(*m)(2, 1) = st;
	(*m)(2, 2) = ct;

	return 0;
}

int		Robot::roty(Matrix *m, double theta)
{
	double ct = cos(theta);
	double st = sin(theta);

	m->setDimension(3, 3);
	(*m)(0, 0) = ct;
	(*m)(0, 1) = 0;
	(*m)(0, 2) = st;
	(*m)(1, 0) = 0;
	(*m)(1, 1) = 1;
	(*m)(1, 2) = 0;
	(*m)(2, 0) = -st;
	(*m)(2, 1) = 0;
	(*m)(2, 2) = ct;

	return 0;
}

int		Robot::rotz(Matrix *m, double theta)
{
	double ct = cos(theta);
	double st = sin(theta);

	m->setDimension(3, 3);
	(*m)(0, 0) = ct;
	(*m)(0, 1) = -st;
	(*m)(0, 2) = 0;
	(*m)(1, 0) = st;
	(*m)(1, 1) = ct;
	(*m)(1, 2) = 0;
	(*m)(2, 0) = 0;
	(*m)(2, 1) = 0;
	(*m)(2, 2) = 1;

	return 0;
}

int		Robot::r2t(Matrix *m)
{
	if (m->getCols() != 3 || m->getRows() != 3)
		return -1;

	Matrix tmp = *m;

	m->setDimension(4, 4);
	(*m)(0, 0) = tmp(0, 0);
	(*m)(0, 1) = tmp(0, 1);
	(*m)(0, 2) = tmp(0, 2);
	(*m)(1, 0) = tmp(1, 0);
	(*m)(1, 1) = tmp(1, 1);
	(*m)(1, 2) = tmp(1, 2);
	(*m)(2, 0) = tmp(2, 0);
	(*m)(2, 1) = tmp(2, 1);
	(*m)(2, 2) = tmp(2, 2);
	(*m)(3, 3) = 1;

	return 0;
}

int		Robot::t2r(Matrix *m)
{
	if (m->getCols() != 4 || m->getRows() != 4)
		return -1;

	Matrix tmp = *m;

	m->setDimension(3, 3);
	(*m)(0, 0) = tmp(0, 0);
	(*m)(0, 1) = tmp(0, 1);
	(*m)(0, 2) = tmp(0, 2);
	(*m)(1, 0) = tmp(1, 0);
	(*m)(1, 1) = tmp(1, 1);
	(*m)(1, 2) = tmp(1, 2);
	(*m)(2, 0) = tmp(2, 0);
	(*m)(2, 1) = tmp(2, 1);
	(*m)(2, 2) = tmp(2, 2);

	return 0;
}

int		Robot::tr2diff(Vector *v, Matrix *t1, Matrix *t2)
{
	v->setLength(6);
	v->Transpose();

	// Position
	(*v)(0) = (*t2)(0, 3) - (*t1)(0, 3);
	(*v)(1) = (*t2)(1, 3) - (*t1)(1, 3);
	(*v)(2) = (*t2)(2, 3) - (*t1)(2, 3);

	// N of NOA matrix
	Vector tmp_1(3);
	tmp_1(0) = (*t1)(0, 0);
	tmp_1(1) = (*t1)(1, 0);
	tmp_1(2) = (*t1)(2, 0);
	Vector tmp_2(3);
	tmp_2(0) = (*t2)(0, 0);
	tmp_2(1) = (*t2)(1, 0);
	tmp_2(2) = (*t2)(2, 0);
	Vector cross_1;
	tmp_1.CrossProd(&cross_1, &tmp_2);

	// O of NOA matrix
	tmp_1(0) = (*t1)(0, 1);
	tmp_1(1) = (*t1)(1, 1);
	tmp_1(2) = (*t1)(2, 1);
	tmp_2(0) = (*t2)(0, 1);
	tmp_2(1) = (*t2)(1, 1);
	tmp_2(2) = (*t2)(2, 1);
	Vector cross_2;
	tmp_1.CrossProd(&cross_2, &tmp_2);

	// A of NOA matrix
	tmp_1(0) = (*t1)(0, 2);
	tmp_1(1) = (*t1)(1, 2);
	tmp_1(2) = (*t1)(2, 2);
	tmp_2(0) = (*t2)(0, 2);
	tmp_2(1) = (*t2)(1, 2);
	tmp_2(2) = (*t2)(2, 2);
	Vector cross_3;
	tmp_1.CrossProd(&cross_3, &tmp_2);

	cross_1.Add(&cross_2);
	cross_1.Add(&cross_3);
	cross_1.Mult(0.5);

	// Orientation
	(*v)(3) = cross_1(0);
	(*v)(4) = cross_1(1);
	(*v)(5) = cross_1(2);

	return 0;
}

int		Robot::tr2diff(Vector *v, Matrix *t)
{
	v->setLength(6);
	v->Transpose();

	// Position
	(*v)(0) = (*t)(0, 3);
	(*v)(1) = (*t)(1, 3);
	(*v)(2) = (*t)(2, 3);
	(*v)(3) = 0.5 * ((*t)(2, 1) - (*t)(1, 2));
	(*v)(4) = 0.5 * ((*t)(0, 2) - (*t)(2, 0));
	(*v)(5) = 0.5 * ((*t)(1, 0) - (*t)(0, 1));

	return 0;
}


/* ROBOT KINEMATICS
********************************/
int		Robot::fKine(Matrix *m, Vector *q)
{
	this->fKineN(m, q, this->DoFs);
	return 0;
}

int		Robot::fKineN(Matrix *m, Vector *q, int n)
{
	if (n > this->DoFs)
		return -1;

	Vector *theta = new Vector(Theta);
	Vector *d = new Vector(D);
	Vector *a = new Vector(A);
	Vector *alpha = new Vector(Alpha);

	for (int i = 0; i < this->DoFs; ++i)
	{
		if (this->Joints[i] == 'r')
		{
			(*theta)(i) += (*q)(i);
		}
		else
			(*d)(i) += (*q)(i);
	}

	m->setDimension(4);
	m->Identity();
	Matrix tmp_denavit;

	for (int j = 0; j < n; ++j)
	{
		this->denavit(&tmp_denavit, (*theta)(j), (*d)(j), (*a)(j), (*alpha)(j));
		m->Mult(&tmp_denavit);
	}

	return 0;
}

int		Robot::Jacob0(Matrix *m, Vector *q)
{
	if (q->getLength() != this->DoFs)
		return -1;

	Matrix m_jN;
	this->JacobN(&m_jN, q);

	Matrix m_fk;
	this->fKine(&m_fk, q);

	Matrix m_j0(6, 6);
	m_j0(0, 0) = m_fk(0, 0);
	m_j0(0, 1) = m_fk(0, 1);
	m_j0(0, 2) = m_fk(0, 2);
	m_j0(1, 0) = m_fk(1, 0);
	m_j0(1, 1) = m_fk(1, 1);
	m_j0(1, 2) = m_fk(1, 2);
	m_j0(2, 0) = m_fk(2, 0);
	m_j0(2, 1) = m_fk(2, 1);
	m_j0(2, 2) = m_fk(2, 2);

	m_j0(3, 3) = m_fk(0, 0);
	m_j0(3, 4) = m_fk(0, 1);
	m_j0(3, 5) = m_fk(0, 2);
	m_j0(4, 3) = m_fk(1, 0);
	m_j0(4, 4) = m_fk(1, 1);
	m_j0(4, 5) = m_fk(1, 2);
	m_j0(5, 3) = m_fk(2, 0);
	m_j0(5, 4) = m_fk(2, 1);
	m_j0(5, 5) = m_fk(2, 2);

	m_j0.Mult(m, &m_jN);

	return 0;
}

int		Robot::JacobN(Matrix *m, Vector *q)
{
	if (q->getLength() != this->DoFs)
		return -1;

	Vector *theta = new Vector(Theta);
	Vector *d = new Vector(D);
	Vector *a = new Vector(A);
	Vector *alpha = new Vector(Alpha);

	for (int i = 0; i < this->DoFs; ++i)
	{
		if (this->Joints[i] == 'r')
		{
			(*theta)(i) += (*q)(i);
		}
		else
			(*d)(i) += (*q)(i);
	}

	Matrix U(4, 4);
	U.Identity();
	Matrix U_tmp = U;
	Matrix m_dh;

	m->setDimension(6, this->DoFs);

	for (int j = (this->DoFs - 1); j >= 0; --j)
	{
		this->denavit(&m_dh, (*theta)(j), (*d)(j), (*a)(j), (*alpha)(j));
		m_dh.Mult(&U, &U_tmp);

		if (this->Joints[j] == 'r')
		{
			(*m)(0, j) = -U(0, 0)*U(1, 3) + U(1, 0)*U(0, 3);
			(*m)(1, j) = -U(0, 1)*U(1, 3) + U(1, 1)*U(0, 3);
			(*m)(2, j) = -U(0, 2)*U(1, 3) + U(1, 2)*U(0, 3);
			(*m)(3, j) = U(2, 0);
			(*m)(4, j) = U(2, 1);
			(*m)(5, j) = U(2, 2);
		}
		else
		{
			(*m)(0, j) = U(2, 0);
			(*m)(1, j) = U(2, 1);
			(*m)(2, j) = U(2, 2);
			(*m)(3, j) = 0;
			(*m)(4, j) = 0;
			(*m)(5, j) = 0;
		}

		U_tmp = U;
	}

	return 0;
}

int		Robot::iKine(Vector *q_out, Matrix *tr, Vector *q)
{
	// Control variables
	double ilimit = 3000;
	double stol = 1 * exp(-6);
	cout << "stol = " << stol << endl;

	double nm = 1;
	int count = 0;

	// Vars to save the tmp results
	Matrix m_fk;
	Vector e;

	if (q->isRow())
		q->Transpose();

	while (nm > stol)
	{

		this->fKine(&m_fk, q);
		this->tr2diff(&e, &m_fk, tr);

	}


	return 0;
}


/* OPERATORS FUNCTIONS
********************************/
ostream& operator<<(ostream& os, const Robot& r)
{
	cout << "| Joint\t| \t Theta \t|\t d \t|\t a \t|\t Alpha \t|" << endl;
	cout << "-------------------------------------------------------------------------\n" << endl;
	for (int i = 0; i < r.DoFs; ++i)
	{
		cout << "| " << i + 1 << "\t|\t";
		if (r.Joints[i] == 'r')
		{
			cout << "q" << i + 1;
			if ((*r.Theta)(i) != 0)
				cout << " + " << (*r.Theta)(i);
		}
		else
			cout << (*r.Theta)(i);

		cout << "\t|\t";

		if (r.Joints[i] == 'p')
		{
			cout << "q" << i + 1;
			if ((*r.D)(i) != 0)
				cout << " + " << (*r.D)(i);
		}
		else
			cout << (*r.D)(i);

		cout << "\t|\t";

		cout << (*r.A)(i);

		cout << "\t|\t";

		cout << (*r.Alpha)(i) << "\t|\n" << endl;

	}

	return os;
}

ostream& operator<<(ostream& os, const Robot* r)
{
	cout << "| Joint\t| \t Theta \t|\t d \t|\t a \t|\t Alpha \t|" << endl;
	cout << "-------------------------------------------------------------------------\n" << endl;
	for (int i = 0; i < r->DoFs; ++i)
	{
		cout << "| " << i + 1 << "\t|\t";
		if (r->Joints[i] == 'r')
		{
			cout << "q" << i + 1;
			if ((*r->Theta)(i) != 0)
				cout << " + " << (*r->Theta)(i);
		}
		else
			cout << (*r->Theta)(i);

		cout << "\t|\t";

		if (r->Joints[i] == 'p')
		{
			cout << "q" << i + 1;
			if ((*r->D)(i) != 0)
				cout << " + " << (*r->D)(i);
		}
		else
			cout << (*r->D)(i);

		cout << "\t|\t";

		cout << (*r->A)(i);

		cout << "\t|\t";

		cout << (*r->Alpha)(i) << "\t|\n" << endl;

	}

	return os;
}

Robot& Robot::operator=(const Robot& rb)
{
	this->A = rb.A;
	this->Alpha = rb.Alpha;
	this->D = rb.D;
	this->Theta = rb.Theta;

	this->Joints = rb.Joints;
	this->DoFs = rb.DoFs;
	
	return *this;
}