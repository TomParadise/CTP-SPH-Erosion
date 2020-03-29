#pragma once
#ifndef INCLUDE_VECTOR3_H_
#define INCLUDE_VECTOR3_H_

#include <cmath>
#include <tuple>

struct Vector3
{
	double x, y, z;
public:
	Vector3()
	{
		x = 0;
		y = 0;
		z = 0;
	}
	Vector3(double val1, double val2, double val3)
	{
		x = val1;
		y = val2;
		z = val3;
	}
	Vector3 scalarMultiply(double val)
	{
		return Vector3(x*val, y*val, z*val);
	}
	Vector3 scalarDivide(double val)
	{
		return Vector3(x/val, y/val, z/val);
	}
	Vector3 vectorMultiply(Vector3 vec)
	{
		return Vector3(x*vec.x, y*vec.y, z*vec.z);
	}
	Vector3 vectorAdd(Vector3 vec)
	{
		return Vector3(x + vec.x, y + vec.y, z + vec.z);
	}
	Vector3 vectorSubtract(Vector3 vec)
	{
		return Vector3(x - vec.x, y - vec.y, z - vec.z);
	}
	double dot(Vector3 vec)
	{
		return x*vec.x + y*vec.y + z*vec.z;
	}
	double length()
	{
		return std::sqrt(x * x + y * y + z * z);
	}
	double lengthSquared()
	{
		return x * x + y * y + z * z;
	}
	double distanceTo(Vector3 vec)
	{
		return this->vectorSubtract(vec).length();
	}
	Vector3 normalized()
	{
		double l = length();
		return Vector3(x / l, y / l, z / l);
	}
	Vector3 cross(Vector3 vec)
	{
		return Vector3(y*vec.z - vec.y*z, z * vec.x - vec.z * x, x * vec.y - vec.x * y);
	}
	std::tuple<Vector3, Vector3> tangential()
	{
		Vector3 a =
			((std::fabs(y) > 0 || std::fabs(z) > 0) ? Vector3(1, 0, 0)
				: Vector3(0, 1, 0))
			.cross(*this)
			.normalized();
		Vector3 b = cross(a);
		return std::make_tuple(a, b);
	}
	double distanceSquaredTo(Vector3 vec)
	{
		return this->vectorSubtract(vec).lengthSquared();
	}
};
#endif