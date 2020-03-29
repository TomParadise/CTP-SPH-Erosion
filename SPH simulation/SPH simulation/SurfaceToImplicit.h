#pragma once
#ifndef INCLUDE_SURFACE_TO_IMPLICIT_H_
#define INCLUDE_SURFACE_TO_IMPLICIT_H_

#include <memory>

#include "ImplicitSurface.h"

//!
//! \brief 3-D implicit surface wrapper for generic Surface3 instance.
//!
//! This class represents 3-D implicit surface that converts Surface3 instance
//! to an ImplicitSurface3 object. The conversion is made by evaluating closest
//! point and normal from a given point for the given (explicit) surface. Thus,
//! this conversion won't work for every single surfaces, especially
//! TriangleMesh3. To use TriangleMesh3 as an ImplicitSurface3 instance,
//! please take a look at ImplicitTriangleMesh3. Use this class only
//! for the basic primitives such as Sphere3 or Box3.
//!
class SurfaceToImplicit : public ImplicitSurface 
{
public:
	//! Constructs an instance with generic Surface3 instance.
	SurfaceToImplicit(const SurfacePtr& surface,
		const Transform& transform = Transform(),
		bool isNormalFlipped = false);

	//! Copy constructor.
	SurfaceToImplicit(const SurfaceToImplicit& other);

	//! Returns the raw surface instance.
	SurfacePtr surface() const;

protected:
	double closestDistanceLocal(Vector3& otherPoint) override;

	BoundingBox boundingBoxLocal() const override;

	double signedDistanceLocal(Vector3& otherPoint)override;

	Vector3 closestPointLocal(Vector3& otherPoint) const override;

	Vector3 closestNormalLocal(Vector3& otherPoint) const override;

	bool isInsideLocal(Vector3 otherPoint) override;
private:
	SurfacePtr _surface;
};

//! Shared pointer for the SurfaceToImplicit3 type.
typedef std::shared_ptr<SurfaceToImplicit> SurfaceToImplicitPtr;

#endif