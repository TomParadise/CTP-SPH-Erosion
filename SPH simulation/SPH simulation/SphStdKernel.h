#pragma once
#ifndef INCLUDE_SPHSTDKERNEL_H_
#define INCLUDE_SPHSTDKERNEL_H_

struct SphStdKernel
{
	double h, h2, h3, h5;

	SphStdKernel();

	explicit SphStdKernel(double kernelRadius);

	//SphStdKernel(const SphStdKernel& other);
	double operator()(double distance) const;

	double firstDerivative(double distance) const;
	Vector3 gradient(double distance, Vector3& direction) const;

	double secondDerivative(double distance) const;

	const double kPiD = 3.14159265358979323846264338327950288;
};

inline SphStdKernel::SphStdKernel() : h(0),h2(0),h3(0){}

inline SphStdKernel::SphStdKernel(double kernelRadius) : h(kernelRadius), h2(h*h), h3(h2*h), h5(h2*h3) {}

inline double SphStdKernel::operator()(double distance) const
{
	if (distance*distance >= h2)
	{
		return 0.0;
	}
	else
	{
		double x = 1.0 - (distance * distance) / h2;
		return 315.0 / (64.0 * kPiD * h3) * x * x * x;
	}
}

inline double SphStdKernel::firstDerivative(double distance) const
{
	if (distance >= h)
	{
		return 0.0;
	}
	else
	{
		double x = 1.0 - (distance * distance) / h2;
		return -945.0 / (32.0*kPiD * h5) * distance * x* x;
	}
}
inline Vector3 SphStdKernel::gradient(double distance, Vector3& direction) const
{
	return direction*(-firstDerivative(distance));
}

inline double SphStdKernel::secondDerivative(double distance) const
{
	if (distance*distance >= h2)
	{
		return 0.0;
	}
	else
	{
		double x = distance * distance / h2;
		return 945.0 / (32.0*kPiD*h5)* (1 - x) * (3 * x - 1);
	}
}
#endif