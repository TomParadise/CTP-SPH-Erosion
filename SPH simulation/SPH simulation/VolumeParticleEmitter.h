#pragma once
#ifndef INCLUDE_VOLUME_PARTICLE_EMITTER_H_
#define INCLUDE_VOLUME_PARTICLE_EMITTER_H_

#include <limits>
#include <memory>
#include <random>

#include "BoundingBox.h"
#include "ImplicitSurface.h"
#include "ParticleEmitter.h"
#include "PointGenerator.h"
#include "BccLatticePointGenerator.h"

class VolumeParticleEmitter : public ParticleEmitter
{
public:
	class Builder;

   //! Constructs an emitter that spawns particles from given implicit surface
   //! which defines the volumetric geometry. Provided bounding box limits
   //! the particle generation region.
   //!
   //! \param[in]  implicitSurface         The implicit surface.
   //! \param[in]  maxRegion               The max region.
   //! \param[in]  spacing                 The spacing between particles.
   //! \param[in]  initialVel              The initial velocity.
   //! \param[in]  linearVel               The linear velocity of the emitter.
   //! \param[in]  angularVel              The angular velocity of the emitter.
   //! \param[in]  maxNumberOfParticles    The max number of particles to be
   //!                                     emitted.
   //! \param[in]  jitter                  The jitter amount between 0 and 1.
   //! \param[in]  isOneShot               True if emitter gets disabled after one shot.
   //! \param[in]  allowOverlapping        True if particles can be overlapped.
   //! \param[in]  seed                    The random seed.
   //!
	VolumeParticleEmitter(
		const ImplicitSurfacePtr& implicitSurface,
		const BoundingBox& maxRegion,
		double spacing,
		const Vector3& initialVel = Vector3(),
		const Vector3& linearVel = Vector3(),
		const Vector3& angularVel = Vector3(),
		size_t maxNumberOfParticles = std::numeric_limits<size_t>::max(),
		double jitter = 0.0,
		bool isOneShot = true,
		bool allowOverlapping = false,
		uint32_t seed = 0);

	void setPointGenerator(const PointGeneratorPtr& newPointsGen);

	//! Returns source surface.
	const ImplicitSurfacePtr& surface() const;

	//! Sets the source surface.
	void setSurface(const ImplicitSurfacePtr& newSurface);

	//! Returns max particle gen region.
	const BoundingBox& maxRegion() const;

	//! Sets the max particle gen region.
	void setMaxRegion(const BoundingBox& newBox);

	//! Returns jitter amount.
	double jitter() const;

	//! Sets jitter amount between 0 and 1.
	void setJitter(double newJitter);

	//! Returns true if particles should be emitted just once.
	bool isOneShot() const;

	//!
	//! \brief      Sets the flag to true if particles are emitted just once.
	//!
	//! If true is set, the emitter will generate particles only once even after
	//! multiple emit calls. If false, it will keep generating particles from
	//! the volumetric geometry. Default value is true.
	//!
	//! \param[in]  newValue True if particles should be emitted just once.
	//!
	void setIsOneShot(bool newValue);

	//! Returns true if particles can be overlapped.
	bool allowOverlapping() const;

	//!
	//! \brief      Sets the flag to true if particles can overlap each other.
	//!
	//! If true is set, the emitter will generate particles even if the new
	//! particles can find existing nearby particles within the particle
	//! spacing.
	//!
	//! \param[in]  newValue True if particles can be overlapped.
	//!
	void setAllowOverlapping(bool newValue);

	//! Returns max number of particles to be emitted.
	size_t maxNumberOfParticles() const;

	//! Sets the max number of particles to be emitted.
	void setMaxNumberOfParticles(size_t newMaxNumberOfParticles);

	//! Returns the spacing between particles.
	double spacing() const;

	//! Sets the spacing between particles.
	void setSpacing(double newSpacing);

	//! Sets the initial velocity of the particles.
	Vector3 initialVelocity() const;

	//! Returns the initial velocity of the particles.
	void setInitialVelocity(const Vector3& newInitialVel);

	//! Returns the linear velocity of the emitter.
	Vector3 linearVelocity() const;

	//! Sets the linear velocity of the emitter.
	void setLinearVelocity(const Vector3& newLinearVel);

	//! Returns the angular velocity of the emitter.
	Vector3 angularVelocity() const;

	//! Sets the linear velocity of the emitter.
	void setAngularVelocity(const Vector3& newAngularVel);

	//! Returns builder fox VolumeParticleEmitter3.
	static Builder builder();

	Vector3 getRandomSpawnPos();

private:
	std::mt19937 _rng;

	ImplicitSurfacePtr _implicitSurface;
	BoundingBox _bounds;
	double _spacing;
	Vector3 _initialVel;
	Vector3 _linearVel;
	Vector3 _angularVel;
	PointGeneratorPtr _pointsGen;

	size_t _maxNumberOfParticles = std::numeric_limits<size_t>::max();
	size_t _numberOfEmittedParticles = 0;

	double _jitter = 0.0;
	bool _isOneShot = true;
	bool _allowOverlapping = false;

	//!
	//! \brief      Emits particles to the particle system data.
	//!
	//! \param[in]  currentTimeInSeconds    Current simulation time.
	//! \param[in]  timeIntervalInSeconds   The time-step interval.
	//!
	void onUpdate(
		double currentTimeInSeconds,
		double timeIntervalInSeconds) override;

	void emit(
		const ParticleSystemDataPtr& particles,
		std::vector<Vector3>* newPositions,
		std::vector<Vector3>* newVelocities);

	double random();

	double random(double min, double max);

	Vector3 velocityAt(Vector3& point);

	Vector3 uniformSampleSphere(float u1, float u2);
};

//! Shared pointer for the VolumeParticleEmitter3 type.
typedef std::shared_ptr<VolumeParticleEmitter> VolumeParticleEmitterPtr;

class VolumeParticleEmitter::Builder
{
public:
	//! Returns builder with implicit surface defining volume shape.
	Builder& withImplicitSurface(const ImplicitSurfacePtr& implicitSurface);

	//! Returns builder with surface defining volume shape.
	Builder& withSurface(const SurfacePtr& surface);

	//! Returns builder with max region.
	Builder& withMaxRegion(const BoundingBox& bounds);

	//! Returns builder with spacing.
	Builder& withSpacing(double spacing);

	//! Returns builder with initial velocity.
	Builder& withInitialVelocity(const Vector3& initialVel);

	//! Returns builder with linear velocity.
	Builder& withLinearVelocity(const Vector3& linearVel);

	//! Returns builder with angular velocity.
	Builder& withAngularVelocity(const Vector3& angularVel);

	//! Returns builder with max number of particles.
	Builder& withMaxNumberOfParticles(size_t maxNumberOfParticles);

	//! Returns builder with jitter amount.
	Builder& withJitter(double jitter);

	//! Returns builder with one-shot flag.
	Builder& withIsOneShot(bool isOneShot);

	//! Returns builder with overlapping flag.
	Builder& withAllowOverlapping(bool allowOverlapping);

	//! Returns builder with random seed.
	Builder& withRandomSeed(uint32_t seed);

	//! Builds VolumeParticleEmitter3.
	VolumeParticleEmitter build() const;

	//! Builds shared pointer of VolumeParticleEmitter3 instance.
	VolumeParticleEmitterPtr makeShared() const;

private:
	ImplicitSurfacePtr _implicitSurface;
	bool _isBoundSet = false;
	BoundingBox _bounds;
	double _spacing = 0.1;
	Vector3 _initialVel;
	Vector3 _linearVel;
	Vector3 _angularVel;
	size_t _maxNumberOfParticles = std::numeric_limits<size_t>::max();
	double _jitter = 0;
	bool _isOneShot = true;
	bool _allowOverlapping = false;
	uint32_t _seed = 0;
};
#endif