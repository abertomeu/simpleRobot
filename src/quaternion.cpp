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
Quaternion::Quaternion(QuaternionStruct *q)
{
	allocSpaceQuat();

	this->quat[0] = q->qs;
	this->quat[1] = q->qx;
	this->quat[2] = q->qy;
	this->quat[3] = q->qz;
}

Quaternion::Quaternion(double qs, double qx, double qy, double qz)
{
	allocSpaceQuat();

	this->quat[0] = qs;
	this->quat[1] = qx;
	this->quat[2] = qy;
	this->quat[3] = qz;
}

Quaternion::Quaternion()
{
	allocSpaceQuat();

	quat[0] = 1;

	for (int i = 1; i < this->quat_size; i++)
	{
		quat[i] = 0;
	}
}

Quaternion::Quaternion(const Quaternion& q) 
{
	allocSpaceQuat();

	for (int i = 0; i < this->quat_size; i++)
	{
		quat[i] = q.quat[i];
	}
}

Quaternion::Quaternion(const Quaternion* q)
{
	allocSpaceQuat();

	for (int i = 0; i < this->quat_size; i++)
	{
		quat[i] = q->quat[i];
	}
}

Quaternion::~Quaternion()
{
	delete[] quat;
}

/* STANDARD QUATERNION
********************************/
int		Quaternion::Identity()
{
	for (int i = 0; i < this->quat_size; i++)
	{
		if (i == 0)
			this->quat[i] = 1;
		else
			this->quat[i] = 0;
	}

	return 0;
}

/* QUATERNION UPDATE MATRIX
********************************/
int Quaternion::Update(QuaternionStruct *q)
{
	this->quat[0] = q->qs;
	this->quat[1] = q->qx;
	this->quat[2] = q->qy;
	this->quat[3] = q->qz;

	return 0;
}

int Quaternion::Update(double qs, double qx, double qy, double qz)
{
	this->quat[0] = qs;
	this->quat[1] = qx;
	this->quat[2] = qy;
	this->quat[3] = qz;

	return 0;
}

/* MATH OPERATIONS MATRIX
********************************/
int		Quaternion::Norm(Quaternion *outQuat)
{
	float sqrt_input = this->quat[0] * this->quat[0] + this->quat[1] * this->quat[1] + this->quat[2] * this->quat[2] + this->quat[3] * this->quat[3];
	float recipNorm;

	invSqrt(&recipNorm, sqrt_input);

	outQuat->quat[0] = this->quat[0] * recipNorm;
	outQuat->quat[1] = this->quat[1] * recipNorm;
	outQuat->quat[2] = this->quat[2] * recipNorm;
	outQuat->quat[3] = this->quat[3] * recipNorm;

	return 0;
}

int		Quaternion::Norm()
{
	float sqrt_input = this->quat[0] * this->quat[0] + this->quat[1] * this->quat[1] + this->quat[2] * this->quat[2] + this->quat[3] * this->quat[3];
	float recipNorm;

	invSqrt(&recipNorm, sqrt_input);

	this->quat[0] = this->quat[0] * recipNorm;
	this->quat[1] = this->quat[1] * recipNorm;
	this->quat[2] = this->quat[2] * recipNorm;
	this->quat[3] = this->quat[3] * recipNorm;

	return 0;
}

int		Quaternion::Conj(Quaternion *outQuat)
{
	for (int i = 0; i < this->quat_size; i++)
	{
		if (i == 0)
			outQuat->quat[i] = this->quat[i];
		else
			outQuat->quat[i] = -this->quat[i];
	}

	return 0;
}

int		Quaternion::Conj()
{
	for (int i = 0; i < this->quat_size; i++)
	{
		if (i == 0)
			this->quat[i] = this->quat[i];
		else
			this->quat[i] = -this->quat[i];
	}

	return 0;
}

int		Quaternion::Prod(Quaternion *outQuat, Quaternion *q)
{
	outQuat->quat[0] = this->quat[0] * q->quat[0] - this->quat[1] * q->quat[1] - this->quat[2] * q->quat[2] - this->quat[3] * q->quat[3];
	outQuat->quat[1] = this->quat[0] * q->quat[1] + this->quat[1] * q->quat[0] + this->quat[2] * q->quat[3] - this->quat[3] * q->quat[2];
	outQuat->quat[2] = this->quat[0] * q->quat[2] - this->quat[1] * q->quat[3] + this->quat[2] * q->quat[0] + this->quat[3] * q->quat[1];
	outQuat->quat[3] = this->quat[0] * q->quat[3] + this->quat[1] * q->quat[2] - this->quat[2] * q->quat[1] + this->quat[3] * q->quat[0];

	return 0;

}

int		Quaternion::Prod(Quaternion *q)
{
	Quaternion *tmp = new Quaternion(*this);

	this->quat[0] = tmp->quat[0] * q->quat[0] - tmp->quat[1] * q->quat[1] - tmp->quat[2] * q->quat[2] - tmp->quat[3] * q->quat[3];
	this->quat[1] = tmp->quat[0] * q->quat[1] + tmp->quat[1] * q->quat[0] + tmp->quat[2] * q->quat[3] - tmp->quat[3] * q->quat[2];
	this->quat[2] = tmp->quat[0] * q->quat[2] - tmp->quat[1] * q->quat[3] + tmp->quat[2] * q->quat[0] + tmp->quat[3] * q->quat[1];
	this->quat[3] = tmp->quat[0] * q->quat[3] + tmp->quat[1] * q->quat[2] - tmp->quat[2] * q->quat[1] + tmp->quat[3] * q->quat[0];

	return 0;
}

/* TRANFORMATION FUNCTIONS
********************************/
Matrix		Quaternion::q2tr()
{
	Matrix tmp(4, 4);
	tmp.Identity();

	double s, x, y, z;
	s = this->quat[0];
	x = this->quat[1];
	y = this->quat[2];
	z = this->quat[3];
	

	tmp(0, 0) = 1 - 2 * (y *y + z * z);
	tmp(0, 1) = 2 * (x*y - s * z);
	tmp(0, 2) = 2 * (x*z + s * y);
	tmp(1, 0) = 2 * (x*y + s * z);
	tmp(1, 1) = 1 - 2 * (x * x + z * z);
	tmp(1, 2) = 2 * (y*z - s * x);
	tmp(2, 0) = 2 * (x*z - s * y);
	tmp(2, 1) = 2 * (y*z + s * x);
	tmp(2, 2) = 1 - 2 * (x * x + y * y);

	return tmp;
}

Matrix	Quaternion::q2r()
{
	Matrix tmp(3, 3);

	double s, x, y, z;
	s = this->quat[0];
	x = this->quat[1];
	y = this->quat[2];
	z = this->quat[3];


	tmp(0, 0) = 1 - 2 * (y *y + z * z);
	tmp(0, 1) = 2 * (x*y - s * z);
	tmp(0, 2) = 2 * (x*z + s * y);
	tmp(1, 0) = 2 * (x*y + s * z);
	tmp(1, 1) = 1 - 2 * (x * x + z * z);
	tmp(1, 2) = 2 * (y*z - s * x);
	tmp(2, 0) = 2 * (x*z - s * y);
	tmp(2, 1) = 2 * (y*z + s * x);
	tmp(2, 2) = 1 - 2 * (x * x + y * y);

	return tmp;
}

Vector	Quaternion::q2Euler()
{
//https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

	Vector tmp(3);

	double s, x, y, z;
	s = this->quat[0];
	x = this->quat[1];
	y = this->quat[2];
	z = this->quat[3];

	// roll (x-axis rotation)
	double sinr_cosp = 2 * (s * x + y * z);
	double cosr_cosp = 1 - 2 * (x * x + y * y);
	tmp(0) = std::atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	double sinp = 2 * (s * y - z * x);
	if (std::abs(sinp) >= 1)
		tmp(1) = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		tmp(1) = std::asin(sinp);

	// yaw (z-axis rotation)
	double siny_cosp = 2 * (s * z + x * y);
	double cosy_cosp = 1 - 2 * (y * y + z * z);
	tmp(2) = std::atan2(siny_cosp, cosy_cosp);

	return tmp;

}

/* OPERATORS FUNCTIONS
********************************/
ostream& operator<<(ostream& os, const Quaternion& q)
{
	os << q.quat[0]<< "\t<\t";
	for (int i = 1; i < q.quat_size-1; ++i) {
		os << q.quat[i];
		os << ",\t";
	}
	os << q.quat[3];
	os << "\t>" << endl;

	return os;
}

ostream& operator<<(ostream& os, const Quaternion* q)
{
	os << q->quat[0] << "\t<\t";
	for (int i = 1; i < q->quat_size - 1; ++i) {
		os << q->quat[i];
		os << ",\t";
	}
	os << q->quat[3];
	os << "\t>" << endl;

	return os;
}

istream& operator>>(istream& is, Quaternion& q)
{
	for (int i = 0; i < q.quat_size; ++i) {
			is >> q.quat[i];
	}
	return is;
}

Quaternion& Quaternion::operator=(const Quaternion& q)
{
	for (int i = 0; i < quat_size; ++i) {
			quat[i] = q.quat[i];
	}

	return *this;
}

/* PRIVATE HELPER FUNCTIONS
********************************/
void	Quaternion::allocSpaceQuat()
{
	quat = new double [quat_size];

}

void	Quaternion::deleteSpaceQuat()
{
	delete[] quat;
}

int		Quaternion::invSqrt(float *out, float x)
{
	long i = 0x5F1F1412 - (*(long*)&x >> 1);
	float tmp = *(float*)&i;
	tmp = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);

	memcpy(out, &tmp, sizeof *out);
	return 0;
}