#include "Plane.h"

Plane::Plane(const Transform & transform, bool isNormalFlipped)
: Surface(transform, isNormalFlipped){}

Plane::Plane(const Vector3 & normal, const Vector3 & point, const Transform & transform, bool isNormalFlipped)
: Surface(transform, isNormalFlipped), normal(normal), point(point) {}

Plane::Plane(const Vector3 & point0, Vector3 & point1, Vector3 & point2, const Transform & transform, bool isNormalFlipped)
: Surface(transform,isNormalFlipped) 
{
	point = point0;
	normal = (point1-point).cross(point2-point).normalized();
}

bool Plane::isBounded() const
{
	return false;
}

Vector3 Plane::closestPointLocal(Vector3 otherPoint) const
{
	Vector3 _normal = normal;
	Vector3 r = otherPoint-point;
	return r - (_normal * _normal.dot(r))  + point;
}

BoundingBox Plane::boundingBoxLocal() const
{
	static const double eps = std::numeric_limits<double>::epsilon();
	static const double dmax = std::numeric_limits<double>::max();

	Vector3 _point = point;
	Vector3 _normal = normal;

	if (std::fabs(_normal.dot(Vector3(1, 0, 0)) - 1.0) < eps) 
	{
		return BoundingBox(_point - Vector3(0, dmax, dmax),
			_point + Vector3(0, dmax, dmax));
	}
	else if (std::fabs(_normal.dot(Vector3(0, 1, 0)) - 1.0) < eps) 
	{
		return BoundingBox(_point - Vector3(dmax, 0, dmax),
			_point + Vector3(dmax, 0, dmax));
	}
	else if (std::fabs(_normal.dot(Vector3(0, 0, 1)) - 1.0) < eps) 
	{
		return BoundingBox(_point - Vector3(dmax, dmax, 0),
			_point + Vector3(dmax, dmax, 0));
	}
	else 
	{
		return BoundingBox(Vector3(dmax, dmax, dmax),
			Vector3(dmax, dmax, dmax));
	}
}

Vector3 Plane::closestNormalLocal(const Vector3 &otherPoint) const
{
	return normal;
}

Plane::Builder& Plane::Builder::withIsNormalFlipped(bool isNormalFlipped) 
{
	_isNormalFlipped = isNormalFlipped;
	return *this;
}

Plane::Builder& Plane::Builder::withTranslation(const Vector3& translation) 
{
	_transform.setTranslation(translation);
	return *this;
}

Plane::Builder& Plane::Builder::withOrientation(const Quaternion& orientation) 
{
	_transform.setOrientation(orientation);
	return *this;
}

Plane::Builder& Plane::Builder::withTransform(const Transform& transform) 
{
	_transform = transform;
	return *this;
}

Plane::Builder & Plane::Builder::withNormal(const Vector3 & normal)
{
	_normal = normal;
	return *this;
}

Plane::Builder & Plane::Builder::withPoint(const Vector3 & point)
{
	_point = point;
	return *this;
}

Plane Plane::Builder::build() const 
{
	return Plane(_normal, _point, _transform, _isNormalFlipped);
}

PlanePtr Plane::Builder::makeShared() const 
{
	return std::shared_ptr<Plane>(
		new Plane(_normal, _point, _transform, _isNormalFlipped),
		[](Plane* obj) { delete obj; });
}