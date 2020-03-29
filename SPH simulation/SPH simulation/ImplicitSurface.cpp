#include "ImplicitSurface.h"


ImplicitSurface::ImplicitSurface(const Transform & transform, bool isNormalFlipped) 
	: Surface(transform, isNormalFlipped){}

ImplicitSurface::ImplicitSurface(const ImplicitSurface& other) : Surface(other){}

ImplicitSurface::ImplicitSurface(const SurfacePtr & surface) : Surface(surface->transform,surface->isNormalFlipped) 
{
	boundingBox() = surface->boundingBox();
}

ImplicitSurface::~ImplicitSurface(){}

double ImplicitSurface::signedDistance(Vector3 & otherPoint)
{
	Vector3 point = transform.toLocal(otherPoint);
	double sd = signedDistanceLocal(point);
	return (isNormalFlipped) ? -sd : sd;
}

double ImplicitSurface::closestDistanceLocal(Vector3 & otherPoint)
{
	return std::fabs(signedDistanceLocal(otherPoint));
}

bool ImplicitSurface::isInsideLocal(Vector3 otherPoint)
{
	return isInsideSdf(signedDistanceLocal(otherPoint));
}

bool ImplicitSurface::isInsideSdf(double phi)
{
	return phi < 0;
}
