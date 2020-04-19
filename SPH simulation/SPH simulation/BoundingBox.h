#pragma once
#ifndef INCLUDE_BOUNDING_BOX_H_
#define INCLUDE_BOUNDING_BOX_H_

#include "Vector3.h"
class BoundingBox
{
public:
	//! Lower corner of the bounding box.
	Vector3 lowerCorner;

	//! Upper corner of the bounding box.
	Vector3 upperCorner;

	BoundingBox();

	//! Constructs a box that tightly covers two points.
	BoundingBox(const Vector3& point1, const Vector3& point2);
	//! Constructs a box with other box instance.
	BoundingBox(const BoundingBox& other);

	//! Returns width of the box.
	double width() const;

	//! Returns height of the box.
	double height() const;

	//! Returns depth of the box.
	double depth() const;

	//! Returns true if the input vector is inside of this box.
	bool contains(const Vector3& point) const;

	//! Expands this box by given delta to all direction.
	//! If the width of the box was x, expand(y) will result a box with
	//! x+y+y width.
	void expand(double delta);

	//! Returns corner position. Index starts from x-first order.
	Vector3 corner(size_t idx) const;

	//! Returns the mid-point of this box.
	Vector3 midPoint() const;

	void merge(const Vector3& point);

	void merge(const BoundingBox& other);

	Vector3 clamp(const Vector3& pt) const;
};

#endif