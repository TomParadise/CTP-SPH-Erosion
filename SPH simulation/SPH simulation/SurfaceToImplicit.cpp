#include "SurfaceToImplicit.h"

SurfaceToImplicit::SurfaceToImplicit(
	const SurfacePtr & surface,
	const Transform& transform,
	bool isNormalFlipped)

	: ImplicitSurface(transform,isNormalFlipped), _surface(surface) {}

SurfaceToImplicit::SurfaceToImplicit(const SurfaceToImplicit & other)
	: ImplicitSurface(other), _surface(other._surface){}

SurfacePtr SurfaceToImplicit::surface() const
{
	return _surface;
}

double SurfaceToImplicit::closestDistanceLocal(Vector3 otherPoint)
{
	return _surface->closestDistance(otherPoint);
}

BoundingBox SurfaceToImplicit::boundingBoxLocal() const
{
	return _surface->boundingBox();
}

double SurfaceToImplicit::signedDistanceLocal(Vector3 & otherPoint)
{
	Vector3 x = _surface->closestPoint(otherPoint);
	bool inside = _surface->isInside(otherPoint);
	return (inside) ? -x.distanceTo(otherPoint) : x.distanceTo(otherPoint);
}

Vector3 SurfaceToImplicit::closestPointLocal(Vector3 otherPoint) const
{
	return _surface->closestPoint(otherPoint);
}

Vector3 SurfaceToImplicit::closestNormalLocal(const Vector3& otherPoint) const
{
	return _surface->closestNormal(otherPoint);
}

bool SurfaceToImplicit::isInsideLocal(Vector3 otherPoint)
{
	return _surface->isInside(otherPoint);
}
