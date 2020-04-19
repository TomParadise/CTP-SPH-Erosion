#include "BoundingBox.h"

#include <algorithm>

BoundingBox::BoundingBox()
{
}

BoundingBox::BoundingBox(const Vector3 & point1, const Vector3 & point2)
{
	lowerCorner.x = std::min(point1.x, point2.x);
	lowerCorner.y = std::min(point1.y, point2.y);
	lowerCorner.z = std::min(point1.z, point2.z);
	upperCorner.x = std::max(point1.x, point2.x);
	upperCorner.y = std::max(point1.y, point2.y);
	upperCorner.z = std::max(point1.z, point2.z);
}

BoundingBox::BoundingBox(const BoundingBox & other)
	: lowerCorner(other.lowerCorner), upperCorner(other.upperCorner) {}

double BoundingBox::width() const
{
	return upperCorner.x - lowerCorner.x;
}

double BoundingBox::height() const
{
	return upperCorner.y - lowerCorner.y;
}


double BoundingBox::depth() const
{
	return upperCorner.z - lowerCorner.z;
}

bool BoundingBox::contains(const Vector3 & point) const
{
	if (upperCorner.x < point.x || lowerCorner.x > point.x) {
		return false;
	}

	if (upperCorner.y < point.y || lowerCorner.y > point.y) {
		return false;
	}

	if (upperCorner.z < point.z || lowerCorner.z > point.z) {
		return false;
	}

	return true;
}

void BoundingBox::expand(double delta)
{
	lowerCorner -= Vector3(delta,delta,delta);
	upperCorner += Vector3(delta, delta, delta);
}

Vector3 BoundingBox::corner(size_t idx) const
{
	double h = 0.5;
	static const Vector3 offset[8] = {
		{-h, -h, -h}, {+h, -h, -h}, {-h, +h, -h}, {+h, +h, -h},
		{-h, -h, +h}, {+h, -h, +h}, {-h, +h, +h}, {+h, +h, +h} };

	return Vector3(width(), height(), depth())*offset[idx] + midPoint();
}

Vector3 BoundingBox::midPoint() const
{
	Vector3 midPoint = upperCorner;
	return (midPoint+lowerCorner)/2;
}

void BoundingBox::merge(const Vector3 & point)
{
	lowerCorner.x = std::min(lowerCorner.x, point.x);
	lowerCorner.y = std::min(lowerCorner.y, point.y);
	lowerCorner.z = std::min(lowerCorner.z, point.z);
	upperCorner.x = std::max(upperCorner.x, point.x);
	upperCorner.y = std::max(upperCorner.y, point.y);
	upperCorner.z = std::max(upperCorner.z, point.z);
}

void BoundingBox::merge(const BoundingBox & other)
{
	lowerCorner.x = std::min(lowerCorner.x, other.lowerCorner.x);
	lowerCorner.y = std::min(lowerCorner.y, other.lowerCorner.y);
	lowerCorner.z = std::min(lowerCorner.z, other.lowerCorner.z);
	upperCorner.x = std::max(upperCorner.x, other.upperCorner.x);
	upperCorner.y = std::max(upperCorner.y, other.upperCorner.y);
	upperCorner.z = std::max(upperCorner.z, other.upperCorner.z);
}

Vector3 BoundingBox::clamp(const Vector3 & pt) const
{
	Vector3 vec;
	vec.x = std::max(lowerCorner.x, pt.x);
	vec.y = std::max(lowerCorner.y, pt.y);
	vec.z = std::max(lowerCorner.z, pt.z);
	vec.x = std::min(upperCorner.x, pt.x);
	vec.y = std::min(upperCorner.y, pt.y);
	vec.z = std::min(upperCorner.z, pt.z);
	return vec;
}

