#include "Collider.h"
#include <algorithm>


Collider::Collider()
{
}


Collider::~Collider()
{
}

void Collider::resolveCollision(double radius, double restitutionCoefficient, Vector3 * position, Vector3 * velocity)
{
	ColliderQueryResult colliderPoint;
	getClosestPoint(_surface, *position, &colliderPoint);

	// Check if the new position is penetrating the surface
	if (isPenetrating(colliderPoint, *position, radius)) {
		// Target point is the closest non-penetrating position from the
		// new position.
		Vector3 targetNormal = colliderPoint.normal;
		Vector3 targetPoint = colliderPoint.point.vectorAdd(targetNormal.scalarMultiply(radius));
		Vector3 colliderVelAtTargetPoint = colliderPoint.velocity;

		// Get new candidate relative velocity from the target point.
		Vector3 relativeVel = velocity->vectorSubtract(colliderVelAtTargetPoint);
		double normalDotRelativeVel = targetNormal.dot(relativeVel);
		Vector3 relativeVelN = targetNormal.scalarMultiply(normalDotRelativeVel);
		Vector3 relativeVelT = relativeVel.vectorSubtract(relativeVelN);

		// Check if the velocity is facing opposite direction of the surface
		// normal
		if (normalDotRelativeVel < 0.0) 
		{
			// Apply restitution coefficient to the surface normal component of
			// the velocity
			Vector3 deltaRelativeVelN =
				relativeVelN.scalarMultiply(-restitutionCoefficient - 1.0);
			relativeVelN = relativeVelN .scalarMultiply(-restitutionCoefficient);

			// Apply friction to the tangential component of the velocity
			if (relativeVelT.lengthSquared() > 0.0) 
			{
				double frictionScale = std::max(
					1.0 - _frictionCoeffient * deltaRelativeVelN.length() /
					relativeVelT.length(),
					0.0);
				relativeVelT = relativeVelT.scalarMultiply(frictionScale);
			}

			// Reassemble the components
			*velocity =
				relativeVelN.vectorAdd(relativeVelT.vectorAdd(colliderVelAtTargetPoint));
		}

		// Geometric fix
		*position = targetPoint;
	}

}

double Collider::frictionCoefficient() const
{
	return _frictionCoeffient;
}

void Collider::setFrictionCoefficient(double newVal)
{
	_frictionCoeffient = std::max(newVal, 0.0);
}

void Collider::update(double currentTimeInSeconds, double timeIntervalInSeconds)
{
	if (_onUpdateCallback) 
	{
		_onUpdateCallback(this, currentTimeInSeconds, timeIntervalInSeconds);
	}
}

void Collider::setOnBeginUpdateCallback(const OnBeginUpdateCallback & callback)
{
	_onUpdateCallback = callback;
}

const SurfacePtr & Collider::surface() const
{
	return _surface;
}

void Collider::setSurface(SurfacePtr& newSurface)
{
	_surface = newSurface;
}

bool Collider::isPenetrating(const ColliderQueryResult & colliderPoint, Vector3 & pos, double radius)
{
	// If the new candidate position of the particle is inside
    // the volume defined by the surface OR the new distance to the surface is
    // less than the particle's radius, this particle is in colliding state.
	return _surface->isInside(pos) || colliderPoint.distance < radius;
}

void Collider::getClosestPoint(SurfacePtr & surface, Vector3 & queryPoint, ColliderQueryResult * result)
{
	result->distance = surface->closestDistance(queryPoint);
	result->point = surface->closestPoint(queryPoint);
	result->normal = surface->closestNormal(queryPoint);
	result->velocity = velocityAt(queryPoint);
}
