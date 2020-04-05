#include "Surface.h"



Surface::Surface(const Transform& transform_, bool isNormalFlipped_)
	: transform(transform_), isNormalFlipped(isNormalFlipped_) {}

Surface::Surface(const Surface& other)
	: transform(other.transform), isNormalFlipped(other.isNormalFlipped) {}

Surface::~Surface() {}

bool Surface::isInside(Vector3 otherPoint) 
{
	return isNormalFlipped == !isInsideLocal(transform.toLocal(otherPoint));
}

double Surface::closestDistance(Vector3 otherPoint)
{
	return closestDistanceLocal(transform.toLocal(otherPoint));
}

Vector3 Surface::closestPoint(Vector3 otherPoint)
{
	return transform.toWorld(closestPointLocal(transform.toLocal(otherPoint)));
}

Vector3 Surface::closestNormal(Vector3 otherPoint)
{
	auto result = transform.toWorldDirection(closestNormalLocal(transform.toLocal(otherPoint)));
	result *= (isNormalFlipped) ? -1.0 : 1.0;
	return result;
}

BoundingBox Surface::boundingBox()
{
	return transform.toWorld(boundingBoxLocal());
}

double Surface::closestDistanceLocal(Vector3 otherPoint)
{
	return otherPoint.distanceTo(closestPointLocal(otherPoint));
}

bool Surface::isInsideLocal(Vector3 otherPointLocal)
{
	Vector3 cpLocal = closestPointLocal(otherPointLocal);
	Vector3 normalLocal = closestNormalLocal(otherPointLocal);
	return (otherPointLocal-cpLocal).dot(normalLocal) < 0.0;
}