#pragma once
#ifndef INCLUDE_PARTICLE_SYSTEM_SOLVER_H_
#define INCLUDE_PARTICLE_SYSTEM_SOLVER_H_

#include <vector>
#include <memory>

#include "PhysicsAnimation.h"
#include "ParticleSystemData.h"
#include "Collider.h"
#include "ParticleEmitter.h"

class ParticleSystemSolver : public PhysicsAnimation
{
public:
	ParticleSystemSolver();
	//! Constructs a solver with particle parameters.
	ParticleSystemSolver(double radius, double mass);
	virtual ~ParticleSystemSolver();
	void resolveCollision();

	const std::shared_ptr<ParticleSystemData> particleSystemData() const;

	//! Returns the collider.
	const ColliderPtr& collider() const;

	//! Sets the collider.
	void setCollider(const ColliderPtr& newCollider);

	const ParticleEmitterPtr& emitter() const;

	void setEmitter(const ParticleEmitterPtr& newEmitter);
	
protected:
	void onAdvanceTimeStep(double timeIntervalInSeconds) override;
	virtual void accumulateForces(double timeStepInSeconds);
	void accumulateExternalForces();

	void resolveCollision(
		std::vector<Vector3>& newPositions,
		std::vector<Vector3>& newVelocities);

	//! Assign a new particle system data.
	void setParticleSystemData(const ParticleSystemDataPtr& newParticles);

	//! Called when a time-step is about to begin.
	virtual void onBeginAdvanceTimeStep(double timeStepInSeconds);

	//! Called after a time-step is completed.
	virtual void onEndAdvanceTimeStep(double timeStepInSeconds);

	void onInitialise() override;

private:
	std::shared_ptr<ParticleSystemData> _particleSystemData;

	void beginAdvanceTimeStep(double timeIntervalInSeconds);
	void endAdvanceTimeStep(double timeIntervalInSeconds);

	void timeIntegration(double timeIntervalInSeconds);

	void updateCollider(double timeStepInSeconds);

	void updateEmitter(double timeStepInSeconds);

	double _dragCoefficient = 1e-4;
	double _restitutionCoefficient = 0.0;
	Vector3 _gravity = Vector3(0.0,-9.8,0.0);

	ParticleSystemData::vectorArray _newPositions;
	ParticleSystemData::vectorArray _newVelocities;
	ColliderPtr _collider;
	ParticleEmitterPtr _emitter;
};

#endif