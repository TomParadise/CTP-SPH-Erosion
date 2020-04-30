#pragma once
#ifndef INCLUDE_QUATERNION_H_
#define INCLUDE_QUATERNION_H_

#include "Vector3.h"
#include "Matrix3x3.h"
class Quaternion
{
public:  
	//! Real part.
	float w;

	//!< Imaginary part (i).
	float x;

	//!< Imaginary part (j).
	float y;

	//!< Imaginary part (k).
	float z;

	//! Make an identity quaternion.
	Quaternion() 
	{
		setIdentity();
	}

	//! Constructs a quaternion with given elements.
	Quaternion(float newW, float newX, float newY, float newZ)
	{
		set(newW, newX, newY, newZ);
	}
	//! Constructs a quaternion with given rotation axis and angle.
	Quaternion(Vector3& axis, float angle)
	{
		set(axis, angle);
	}

	//! Constructs a quaternion with from and to vectors.
	Quaternion(Vector3& from, Vector3& to)
	{
		set(from, to);
	}

	//! Copy constructor.
	Quaternion(Quaternion& other)
	{
		set(other);
	}

	//! Sets the quaternion with given elements.
	void set(float newW, float newX, float newY, float newZ)
	{
		w = newW;
		x = newX;
		y = newY;
		z = newZ;
	}

	//! Sets the quaternion with other quaternion.
	void set(Quaternion& other)
	{
		set(other.w, other.x, other.y, other.z);
	}

	//! Sets the quaternion with given rotation axis and angle.
	void set(Vector3& axis, float angle)
	{
		static const float eps = std::numeric_limits<float>::epsilon();
		float axisLengthSquared = axis.lengthSquared();

		if (axisLengthSquared < eps)
		{
			setIdentity();
		}
		else
		{
			Vector3 normalizedAxis = axis.normalized();
			float s = std::sin(angle / 2);

			x = normalizedAxis.x * s;
			y = normalizedAxis.y * s;
			z = normalizedAxis.z * s;
			w = std::cos(angle / 2);
		}
	}

	//! Sets the quaternion with from and to vectors.
	void set(Vector3& from, Vector3& to)
	{
		static const float eps = std::numeric_limits<float>::epsilon();

		Vector3 axis = from.cross(to);

		float fromLengthSquared = from.lengthSquared();
		float toLengthSquared = to.lengthSquared();

		if (fromLengthSquared < eps ||
			toLengthSquared < eps) {
			setIdentity();
		}
		else {
			float axisLengthSquared = axis.lengthSquared();

			// In case two vectors are exactly the opposite, pick orthogonal vector
			// for axis.
			if (axisLengthSquared < eps) {
				axis = std::get<0>(from.tangential());
			}

			set(from.dot(to), axis.x, axis.y, axis.z);
			w += l2Norm();

			normalize();
		}
	}
	//! Makes this quaternion identity.
	void setIdentity()
	{
		set(1, 0, 0, 0);
	}

	//! Returns the inverse quaternion.
	Quaternion inverse() const
	{
		float denom = w * w + x * x + y * y + z * z;
		Quaternion quat(w / denom, -x / denom, -y / denom, -z / denom);
		return quat;
	}

	//! Converts to the 3x3 rotation matrix.
	Matrix3x3 matrix3() const
	{
		float _2xx = 2 * x * x;
		float _2yy = 2 * y * y;
		float _2zz = 2 * z * z;
		float _2xy = 2 * x * y;
		float _2xz = 2 * x * z;
		float _2xw = 2 * x * w;
		float _2yz = 2 * y * z;
		float _2yw = 2 * y * w;
		float _2zw = 2 * z * w;

		Matrix3x3 m(
			1 - _2yy - _2zz, _2xy - _2zw, _2xz + _2yw,
			_2xy + _2zw, 1 - _2zz - _2xx, _2yz - _2xw,
			_2xz - _2yw, _2yz + _2xw, 1 - _2yy - _2xx);

		return m;
	}

	//! Returns L2 norm of this quaternion.
	float l2Norm() const
	{
		return std::sqrt(w * w + x * x + y * y + z * z);
	}

	//! Normalizes the quaternion.
	void normalize()
	{
		float norm = l2Norm();

		if (norm > 0) {
			w /= norm;
			x /= norm;
			y /= norm;
			z /= norm;
		}
	}
};

#endif