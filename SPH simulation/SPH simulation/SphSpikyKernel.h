#pragma once
#ifndef INCLUDE_SPHSPIKYKERNEL_H_
#define INCLUDE_SPHSPIKYKERNEL_H_

#include "Vector3.h"

struct SphSpikyKernel
{
	double h, h2, h3, h4, h5;
	SphSpikyKernel();

	explicit SphSpikyKernel(double kernelRadius);

	double operator()(double distance) const;

	double firstDerivative(double distance) const;

	Vector3 gradient(double distance, Vector3& direction) const;

	double secondDerivative(double distance) const;

	const double kPiD = 3.14159265358979323846264338327950288;
};

inline SphSpikyKernel::SphSpikyKernel()
	: h(0), h2(0), h3(0), h4(0), h5(0){}

inline SphSpikyKernel::SphSpikyKernel(double h_)
	: h(h_), h2(h*h), h3(h2*h), h4(h2*h2), h5(h3*h2) {}

inline double SphSpikyKernel::operator()(double distance) const
{
	if (distance >= h)
	{
		return 0.0;
	}
	else
	{
		double x = 1.0 - distance / h;
		return 15.0 / (kPiD*h3)*x*x*x;
	}
}

inline double SphSpikyKernel::firstDerivative(double distance) const
{
	if (distance >= h)
	{
		return 0.0;
	}
	else
	{
		double x = 1.0 - distance / h;
		return -45.0 / (kPiD*h4)*x*x;
	}
}

inline Vector3 SphSpikyKernel::gradient(double distance, Vector3 & direction) const
{
	return direction*(-firstDerivative(distance));
}

inline double SphSpikyKernel::secondDerivative(double distance) const
{
	if (distance >= h)
	{
		return 0.0;
	}
	else
	{
		double x = 1.0 - distance / h;
		return 90.0 / (kPiD*h5)*x;
	}
}
#endif