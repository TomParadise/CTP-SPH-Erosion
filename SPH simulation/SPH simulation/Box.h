#pragma once
#ifndef INCLUDE_BOX_H_
#define INCLUDE_BOX_H_

#include "Surface.h"
#include "BoundingBox.h"
#include "Plane.h"

class Box : public Surface
{
public:
	class Builder;

	BoundingBox bound = BoundingBox(Vector3(), Vector3(1, 1, 1));

	//! Constructs (0, 0, 0) x (1, 1, 1) box.
	Box(
		const Transform& transform = Transform(),
		bool isNormalFlipped = false);

	//! Constructs a box with given \p lowerCorner and \p upperCorner.
	Box(
		const Vector3& lowerCorner,
		const Vector3& upperCorner,
		const Transform& transform = Transform(),
		bool isNormalFlipped = false);

	//! Constructs a box with BoundingBox instance.
	explicit Box(
		const BoundingBox& boundingBox,
		const Transform& transform = Transform(),
		bool isNormalFlipped = false);

	//! Returns builder fox Box.
	static Builder builder();

protected:
	// Surface3 implementations

	Vector3 closestPointLocal(Vector3& otherPoint) const override;

	BoundingBox boundingBoxLocal() const;

	Vector3 closestNormalLocal(Vector3& otherPoint) const override;
};

//! Shared pointer type for the Box3.
typedef std::shared_ptr<Box> BoxPtr;

class Box::Builder
{
public:
	Builder& withIsNormalFlipped(bool isNormalFlipped);

	//! Returns builder with translation.
	Builder& withTranslation(const Vector3& translation);

	//! Returns builder with orientation.
	Builder& withOrientation(const Quaternion& orientation);

	//! Returns builder with transform.
	Builder& withTransform(const Transform& transform);

	//! Returns builder with lower corner set.
	Builder& withLowerCorner(const Vector3& pt);

	//! Returns builder with upper corner set.
	Builder& withUpperCorner(const Vector3& pt);

	//! Returns builder with bounding box.
	Builder& withBoundingBox(const BoundingBox& bbox);

	//! Builds Box3.
	Box build() const;

	//! Builds shared pointer of Box3 instance.
	BoxPtr makeShared() const;

private:
	bool _isNormalFlipped = false;
	Transform _transform;
	Vector3 _lowerCorner{ 0, 0, 0 };
	Vector3 _upperCorner{ 1, 1, 1 };
};

#endif