#pragma once
#ifndef INCLUDE_IMPLICIT_SURFACE_H_
#define INCLUDE_IMPLICIT_SURFACE_H_

#include "Surface.h"

class ImplicitSurface : public Surface
{
public:
	//! Default constructor.
	ImplicitSurface(
		const Transform& transform = Transform(),
		bool isNormalFlipped = false);

	//! Copy constructor.
	ImplicitSurface(const ImplicitSurface& other);

	ImplicitSurface(const SurfacePtr& surface);

	virtual ~ImplicitSurface();

	//! Returns signed distance from the given point \p otherPoint.
	double signedDistance(Vector3& otherPoint);

protected:
	//! Returns signed distance from the given point \p otherPoint in local
	//! space.
	virtual double signedDistanceLocal(Vector3& otherPoint) = 0;

private:
	double closestDistanceLocal(Vector3& otherPoint) override;

	bool isInsideLocal(Vector3 otherPoint) override;

	bool isInsideSdf(double phi);
};

//! Shared pointer type for the ImplicitSurface3.
typedef std::shared_ptr<ImplicitSurface> ImplicitSurfacePtr;

#endif