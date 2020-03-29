#pragma once
#ifndef INCLUDE_TRANSFORM_H_
#define INCLUDE_TRANSFORM_H_

#include "Vector3.h"
#include "Matrix3x3.h"
#include "Quaternion.h"
#include "BoundingBox.h"

class Transform
{
public:
	//! Constructs identity transform.
	Transform();

	//! Constructs a transform with translation and orientation.
	Transform(const Vector3& translation, const Quaternion& orientation);

	//! Constructs a transform with translation and orientation.
	Transform(const Transform& transform);

	//! Returns the translation.
	const Vector3& translation() const;

	//! Sets the traslation.
	void setTranslation(const Vector3& translation);

	//! Transforms a point in world coordinate to the local frame.
	Vector3 toLocal(Vector3& pointInWorld);
	
	//! Returns the orientation.
	const Quaternion& orientation() const;

	//! Sets the orientation.
	void setOrientation(const Quaternion& orientation);

	Vector3 translation();

	Quaternion orientation();

	Matrix3x3 orientationMatrix() const;

	Matrix3x3 inverseOrientationMatrix() const;

	//! Transforms a point in local space to the world coordinate.
	Vector3 toWorld(Vector3 pointInLocal);

	//! Transforms a bounding box in local space to the world coordinate.
	BoundingBox toWorld(const BoundingBox& bboxInLocal);

	//! Transforms a direction in local space to the world coordinate.
	Vector3 toWorldDirection(Vector3& dirInLocal);
private:
	Vector3 _translation;
	Quaternion _orientation;
	Matrix3x3 _orientationMat3;
	Matrix3x3 _inverseOrientationMat3;
};
#endif