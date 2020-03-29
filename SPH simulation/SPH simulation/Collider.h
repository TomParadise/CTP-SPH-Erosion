#pragma once
#ifndef INCLUDE_COLLIDER_H_
#define INCLUDE_COLLIDER_H_

#include <functional>
#include <memory>

#include "Surface.h"
#include "Vector3.h"
class Collider
{
public:
	typedef std::function<void(Collider*, double, double)>
		OnBeginUpdateCallback;

	Collider();
	virtual ~Collider();

	virtual Vector3 velocityAt(Vector3& point) = 0;

	void resolveCollision(
		double radius,
		double restitutionCoefficient,
		Vector3* position,
		Vector3* velocity);

	double frictionCoefficient() const;
	void setFrictionCoefficient(double newVal);

	void update(double  currentTimeInSeconds, double timeIntervalInSeconds);

	void setOnBeginUpdateCallback(const OnBeginUpdateCallback& callback);

	const SurfacePtr& surface() const;

protected:
	struct ColliderQueryResult final {
		double distance;
		Vector3 point;
		Vector3 normal;
		Vector3 velocity;
	};

	void setSurface(SurfacePtr& newSurface);

	bool isPenetrating(
		const ColliderQueryResult& colliderPoint,
		Vector3& pos,
		double radius);

	void getClosestPoint(
		SurfacePtr& surface,
		Vector3& queryPoint,
		ColliderQueryResult* result);

private:
	double _frictionCoeffient = 0.0;
	OnBeginUpdateCallback _onUpdateCallback;
	SurfacePtr _surface;
};

typedef std::shared_ptr<Collider> ColliderPtr;

#endif