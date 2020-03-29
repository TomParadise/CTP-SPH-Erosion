#include "RigidBodyCollider.h"

RigidBodyCollider::RigidBodyCollider(SurfacePtr & surface)
{
	setSurface(surface);
}

RigidBodyCollider::RigidBodyCollider(SurfacePtr & surface, const Vector3 & linearVelocity, const Vector3 & angularVelocity)
	: linearVelocity(linearVelocity)
	, angularVelocity(angularVelocity)
{
	setSurface(surface);
}

Vector3 RigidBodyCollider::velocityAt(Vector3 & point)
{
	Vector3 r = point.vectorSubtract(surface()->transform.translation());
	return linearVelocity.vectorAdd(angularVelocity.cross(r));
}

RigidBodyCollider::Builder RigidBodyCollider::builder()
{
	return Builder();
}

RigidBodyCollider::Builder& RigidBodyCollider::Builder::withSurface(const SurfacePtr & surface)
{
	_surface = surface;
	return *this;
}
RigidBodyCollider::Builder& RigidBodyCollider::Builder::withLinearVelocity(const Vector3& linearVelocity)
{
	_linearVelocity = linearVelocity;
	return *this;
}

RigidBodyCollider::Builder& RigidBodyCollider::Builder::withAngularVelocity(const Vector3& angularVelocity)
{
	_angularVelocity = angularVelocity;
	return *this;
}

RigidBodyCollider  RigidBodyCollider::Builder::build()
{
	return RigidBodyCollider(
		_surface,
		_linearVelocity,
		_angularVelocity);
}

RigidBodyColliderPtr  RigidBodyCollider::Builder::makeShared()
{
	return std::shared_ptr<RigidBodyCollider>(
		new RigidBodyCollider(
			_surface,
			_linearVelocity,
			_angularVelocity),
		[](RigidBodyCollider* obj) {
		delete obj;
	});
}