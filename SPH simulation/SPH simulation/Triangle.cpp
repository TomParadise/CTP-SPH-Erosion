#include "Triangle.h"

inline Vector3 closestPointOnLine(const Vector3& v0, const Vector3& v1, const Vector3& pt) 
{
	double lenSquared = (v1 - v0).lengthSquared();
	if (lenSquared < std::numeric_limits<double>::epsilon()) 
	{
		return v0;
	}

	double t = (pt - v0).dot(v1 - v0) / lenSquared;
	if (t < 0.0)
	{
		return v0;
	}
	else if (t > 1.0) 
	{
		return v1;
	}

	return v0 + (v1 - v0) * t;
}

inline Vector3 closestNormalOnLine(const Vector3& v0, const Vector3& v1, const Vector3& n0, const Vector3& n1, const Vector3& pt) 
{
	 double lenSquared = (v1 - v0).lengthSquared();
	if (lenSquared < std::numeric_limits<double>::epsilon()) 
	{
		return n0;
	}

	const double t = (pt - v0).dot(v1 - v0) / lenSquared;
	if (t < 0.0) 
	{
		return n0;
	}
	else if (t > 1.0)
	{
		return n1;
	}

	return (n0 + (n1 - n0) * t).normalized();
}

Triangle::Triangle(const Transform & transform, bool isNormalFlipped)
	: Surface(transform, isNormalFlipped) {}

Triangle::Triangle(const std::array<Vector3, 3>& points, const std::array<Vector3, 3>& normals, const Transform & transform, bool isNormalFlipped)
	: Surface(transform, isNormalFlipped),
	points(points),
	normals(normals) {}

Vector3 Triangle::faceNormal() const
{
	Vector3 ret = (points[1] - points[0]).cross(points[2] - points[0]);
	return ret.normalized();
}

double Triangle::area() const
{
	return 0.5 * (points[1] - points[0]).cross(points[2] - points[0]).length();
}

Vector3 Triangle::closestPointLocal(Vector3 otherPoint) const
{
	Vector3 n = faceNormal();
	double nd = n.dot(n);
	double d = n.dot(points[0]);
	double t = (d - n.dot(otherPoint)) / nd;

	Vector3 q = n *t + otherPoint;

	Vector3 q01 = (points[1] - points[0]).cross(q - points[0]);
	if (n.dot(q01) < 0) {
		return closestPointOnLine(points[0], points[1], q);
	}

	Vector3 q12 = (points[2] - points[1]).cross(q - points[1]);
	if (n.dot(q12) < 0) {
		return closestPointOnLine(points[1], points[2], q);
	}

	Vector3 q02 = (points[0] - points[2]).cross(q - points[2]);
	if (n.dot(q02) < 0) {
		return closestPointOnLine(points[0], points[2], q);
	}

	double a = area();
	double b0 = 0.5 * q12.length() / a;
	double b1 = 0.5 * q02.length() / a;
	double b2 = 0.5 * q01.length() / a;

	return points[0] * b0 + points[1] * b1 + points[2] * b2;
}

BoundingBox Triangle::boundingBoxLocal() const
{
	BoundingBox box(points[0], points[1]);
	box.merge(points[2]);
	return box;
}

Vector3 Triangle::closestNormalLocal(const Vector3 & otherPoint) const
{
	Vector3 n = faceNormal();
	double nd = n.dot(n);
	double d = n.dot(points[0]);
	double t = (d - n.dot(otherPoint)) / nd;

	Vector3 q = n * t + otherPoint;

	Vector3 q01 = (points[1] - points[0]).cross(q - points[0]);
	if (n.dot(q01) < 0) {
		return closestNormalOnLine(points[0], points[1], normals[0], normals[1],
			q);
	}

	Vector3 q12 = (points[2] - points[1]).cross(q - points[1]);
	if (n.dot(q12) < 0) {
		return closestNormalOnLine(points[1], points[2], normals[1], normals[2],
			q);
	}

	Vector3 q02 = (points[0] - points[2]).cross(q - points[2]);
	if (n.dot(q02) < 0) {
		return closestNormalOnLine(points[0], points[2], normals[0], normals[2],
			q);
	}

	double a = area();
	double b0 = 0.5 * q12.length() / a;
	double b1 = 0.5 * q02.length() / a;
	double b2 = 0.5 * q01.length() / a;

	return (normals[0] * b0 + normals[1] * b1 + normals[2] * b2).normalized();
}
