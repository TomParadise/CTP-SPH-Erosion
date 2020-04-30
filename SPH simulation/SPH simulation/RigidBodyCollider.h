#pragma once
#ifndef INCLUDE_RIGID_BODY_COLLIDER_H_
#define INCLUDE_RIGID_BODY_COLLIDER_H_

#include "Collider.h"
#include "Quaternion.h"

class RigidBodyCollider : public Collider
{
public:
	class Builder;

	//! Linear velocity of the rigid body.
	Vector3 linearVelocity;

	//! Angular velocity of the rigid body.
	Vector3 angularVelocity;

	//! Constructs a collider with a surface.
	explicit RigidBodyCollider(SurfacePtr& surface);

	//! Constructs a collider with a surface and other parameters.
	RigidBodyCollider(
		SurfacePtr& surface,
		const Vector3& linearVelocity,
		const Vector3& angularVelocity);

	//! Returns the velocity of the collider at given \p point.
	Vector3 velocityAt(Vector3& point) override;

	//! Returns builder fox RigidBodyCollider3.
	static Builder builder();
};

typedef std::shared_ptr<RigidBodyCollider> RigidBodyColliderPtr;

class RigidBodyCollider::Builder 
{
public:
	//! Returns builder with surface.
	Builder& withSurface(const SurfacePtr& surface);

	//! Returns builder with linear velocity.
	Builder& withLinearVelocity(const Vector3& linearVelocity);

	//! Returns builder with angular velocity.
	Builder& withAngularVelocity(const Vector3& angularVelocity);

	//! Builds RigidBodyCollider3.
	RigidBodyCollider build();

	//! Builds shared pointer of RigidBodyCollider3 instance.
	RigidBodyColliderPtr makeShared();

private:
	SurfacePtr _surface;
	Vector3 _linearVelocity{ 0, 0, 0 };
	Vector3 _angularVelocity{ 0, 0, 0 };
};
#endif