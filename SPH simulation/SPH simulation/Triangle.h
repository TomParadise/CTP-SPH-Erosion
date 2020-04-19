#pragma once

#ifndef INCLUDE_TRIANGLE_H_
#define INCLUDE_TRIANGLE_H_

#include <vector>

#include "Surface.h"

class Triangle : public Surface
{
public:
	//! Three points.
	std::array<Vector3,3> points;

	//! Three normals.
	std::array<Vector3,3> normals;

	//! Constructs an empty triangle.
	Triangle(
		const Transform& transform = Transform(),
		bool isNormalFlipped = false);

	//! Constructs a triangle with given \p points, \p normals, and \p uvs.
	Triangle(
		const std::array<Vector3, 3>& points,
		const std::array<Vector3, 3>& normals,
		const Transform& transform = Transform(),
		bool isNormalFlipped = false);
	
	//! Returns the face normal of the triangle.
	Vector3 faceNormal() const;

	double area() const;

protected:
	Vector3 closestPointLocal(Vector3 otherPoint) const override;


	BoundingBox boundingBoxLocal() const override;

	Vector3 closestNormalLocal(const Vector3& otherPoint) const override;
};

//! Shared pointer for the Triangle3 type.
typedef std::shared_ptr<Triangle> TrianglePtr;
#endif