#pragma once
#ifndef INCLUDE_SURFACE_H_
#define INCLUDE_SURFACE_H_

#include <limits>
#include "Vector3.h"
#include "Transform.h"
#include "BoundingBox.h"

//! Struct that represents ray-surface intersection point.
struct SurfaceRayIntersection3
{
	bool isIntersecting = false;
	double distance = std::numeric_limits<double>::max();
	Vector3 point;
	Vector3 normal;
};

class Surface
{
public:
	//! Local-to-world transform.
	Transform transform;

	//! Flips normal when calling Surface3::closestNormal(...).
	bool isNormalFlipped = false;

	//! Constructs a surface with normal direction.
	Surface(const Transform& transform = Transform(),
		bool isNormalFlipped = false);

	//! Copy constructor.
	Surface(const Surface& other);

	//! Default destructor.
	virtual ~Surface();

	//! Returns true if \p otherPoint is inside the volume defined by the
	//! surface.
	bool isInside(Vector3 otherPoint);
	   
	//! Returns the closest distance from the given point \p otherPoint to the
	//! point on the surface.
	double closestDistance(Vector3 otherPoint);

	//! Returns the closest point from the given point \p otherPoint to the
	//! surface.
	Vector3 closestPoint(Vector3 otherPoint);

	//! Returns the normal to the closest point on the surface from the given
	//! point \p otherPoint.
	Vector3 closestNormal(Vector3 otherPoint);

	//! Returns the bounding box of this surface object.
	BoundingBox boundingBox();

	//! Updates internal spatial query engine.
	virtual void updateQueryEngine();

protected:
	//! Returns the closest point from the given point \p otherPoint to the
	//! surface in local frame.
	virtual Vector3 closestPointLocal(Vector3 otherPoint) const = 0;

	//! Returns the normal to the closest point on the surface from the given
	//! point \p otherPoint in local frame.
	virtual Vector3 closestNormalLocal(const Vector3& otherPoint) const = 0;

	//! Returns the closest distance from the given point \p otherPoint to the
	//! point on the surface in local frame.
	virtual double closestDistanceLocal(Vector3 otherPoint);

	//! Returns true if \p otherPoint is inside by given \p depth the volume
	//! defined by the surface in local frame.
	virtual bool isInsideLocal(Vector3 otherPoint);

	//! Returns the bounding box of this surface object in local frame.
	virtual BoundingBox boundingBoxLocal() const = 0;
};

typedef std::shared_ptr<Surface> SurfacePtr;
#endif