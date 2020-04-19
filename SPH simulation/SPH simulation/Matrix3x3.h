#pragma once
#ifndef INCLUDE_MATRIX3X3_H_
#define INCLUDE_MATRIX3X3_H_

#include "Vector3.h"

#include <array>
#include <limits>

struct Matrix3x3
{
	float m00, m01, m02,
			m10, m11, m12,
			m20, m21, m22;
public:
	Matrix3x3()
	{
		m00 = 1;
		m01 = 0;
		m02 = 0;
		m10 = 0;
		m11 = 1;
		m12 = 0;
		m20 = 0;
		m21 = 0;
		m22 = 1;
	}
	Matrix3x3(float _m00, float _m01, float _m02,
			  float _m10, float _m11, float _m12, 
			  float _m20, float _m21, float _m22)
	{
		m00 = _m00;
		m01 = _m01;
		m02 = _m02;
		m10 = _m10;
		m11 = _m11;
		m12 = _m12;
		m20 = _m20;
		m21 = _m21;
		m22 = _m22;
	}

	//! Sets i-th row with input vector.
	void setRow(size_t i, const float& row)
	{
		if (i == 1)
		{
			m00 = row;
			m10 = row;
			m20 = row;
		}
		else if (i == 2)
		{
			m01 = row;
			m11 = row;
			m21 = row;
		}
		else if(i == 3)
		{
			m02 = row;
			m12 = row;
			m22 = row;
		}
	}

	//! Sets i-th column with input vector.
	void setColumn(size_t i, const float& col)
	{
		if (i == 1)
		{
			m00 = col;
			m01 = col;
			m02 = col;
		}
		else if (i == 2)
		{
			m10 = col;
			m11 = col;
			m12 = col;
		}
		else if (i == 3)
		{
			m20 = col;
			m21 = col;
			m22 = col;
		}
	}

	//! Sets i-th column with input vector.
	void setElement(size_t col, size_t row, const float& val)
	{
		if (col == 1)
		{
			if (row == 1)
			{
				m00 = val;
			}
			else if (row == 2)
			{
				m10 = val;
			}
			else if (row == 3)
			{
				m20 = val;
			}
		}
		else if (col == 2)
		{
			if (row == 1)
			{
				m01 = val;
			}
			else if (row == 2)
			{
				m11 = val;
			}
			else if (row == 3)
			{
				m21 = val;
			}
		}
		else if (col == 3)
		{
			if (row == 1)
			{
				m02 = val;
			}
			else if (row == 2)
			{
				m12 = val;
			}
			else if (row == 3)
			{
				m22 = val;
			}
		}
	}

	Vector3 vectorMultiply(Vector3 vec)
	{
		return Vector3(
			(vec.x * m00 + vec.y * m01 + vec.z * m02),
			(vec.x * m10 + vec.y * m11 + vec.z * m12),
			(vec.x * m20 + vec.y * m21 + vec.z * m22));
	}

	float determinant() const
	{
		return m00 * m11 * m22 -
			m00 * m12 * m21 +
			m01 * m12 * m20 -
			m01 * m10 * m22 +
			m02 * m10 * m21 -
			m02 * m11 * m20;
	}
};
#endif