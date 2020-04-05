#pragma once
#ifndef INCLUDE_PLANE_H_
#define INCLUDE_PLANE_H_

#include "Surface.h"
#include "BoundingBox.h"

class Plane : public Surface
{
public:
	class Builder;

	Vector3 normal = Vector3(0, 1, 0);
	Vector3 point;

	//! Constructs a plane that crosses (0, 0, 0) with surface normal (0, 1, 0).
	Plane(const Transform& transform = Transform(),
		bool isNormalFlipped = false); 

	//! Constructs a plane that cross \p point with surface normal \p normal.
	Plane(
		const Vector3& normal,
		const Vector3& point,
		const Transform& transform = Transform(),
		bool isNormalFlipped = false);

	//! Constructs a plane with three points on the surface. The normal will be
	//! set using the counter clockwise direction.
	Plane(
		const Vector3& point0,
		Vector3& point1,
		Vector3& point2,
		const Transform& transform = Transform(),
		bool isNormalFlipped = false);

	//! Returns true if bounding box can be defined.
	bool isBounded() const;
protected:
		Vector3 closestPointLocal(Vector3 otherPoint) const override;

		BoundingBox boundingBoxLocal() const;

		Vector3 closestNormalLocal(const Vector3& otherPoint) const override;
};

//! Shared pointer for the Plane3 type.
typedef std::shared_ptr<Plane> PlanePtr;

class Plane::Builder
{
 public:   
	 //! Returns builder with flipped normal flag.
	 Builder& withIsNormalFlipped(bool isNormalFlipped);

	 //! Returns builder with translation.
	 Builder& withTranslation(const Vector3& translation);

	 //! Returns builder with orientation.
	 Builder& withOrientation(const Quaternion& orientation);

	 //! Returns builder with transform.
	 Builder& withTransform(const Transform& transform);
	 //! Returns builder with plane normal.
	 Builder& withNormal(const Vector3& normal);

	 //! Returns builder with point on the plane.
	 Builder& withPoint(const Vector3& point);

	 //! Builds Plane3.
	 Plane build() const;

	 //! Builds shared pointer of Plane3 instance.
	 PlanePtr makeShared() const;

  private:

	 bool _isNormalFlipped = false;
	 Transform _transform;
	 Vector3 _normal{0, 1, 0};
	 Vector3 _point{0, 0, 0};
};

#endif