#include "ParticleSystemSolver.h"

ParticleSystemSolver::ParticleSystemSolver()
	: ParticleSystemSolver(1e-3, 1e-3){}

ParticleSystemSolver::ParticleSystemSolver(double radius, double mass)
{
	_particleSystemData = std::make_shared<ParticleSystemData>();
	_particleSystemData->setRadius(radius);
	_particleSystemData->setMass(mass);	
}

ParticleSystemSolver::~ParticleSystemSolver()
{
}

void ParticleSystemSolver::setEmitter(const ParticleEmitterPtr & newEmitter)
{
	_emitter = newEmitter;
	newEmitter->setTarget(_particleSystemData);
}

void ParticleSystemSolver::onAdvanceTimeStep(double timeIntervalInSeconds)
{
	beginAdvanceTimeStep(timeIntervalInSeconds);

	accumulateForces(timeIntervalInSeconds);
	timeIntegration(timeIntervalInSeconds);
	resolveCollision();

	endAdvanceTimeStep(timeIntervalInSeconds);
}

void ParticleSystemSolver::accumulateForces(double timeStepInSeconds)
{
	accumulateExternalForces();
}

void ParticleSystemSolver::accumulateExternalForces()
{
	size_t n = _particleSystemData->numberOfParticles();
	auto velocities = _particleSystemData->velocities();
	auto positions = _particleSystemData->positions();
	const double mass = _particleSystemData->mass();

	for (size_t i = 0; i < n; i++)
	{
		//gravity
		Vector3 force = _gravity.scalarMultiply(mass);

		force = force.vectorAdd(_particleSystemData->velocities()[i].scalarMultiply(-_dragCoefficient));

		if (i >= _particleSystemData->forces().size())
		{
			_particleSystemData->forces().push_back(force);
		}
		else
		{
			_particleSystemData->forces()[i] = _particleSystemData->forces()[i].vectorAdd(force);
		}
	}
}

void ParticleSystemSolver::resolveCollision()
{
	resolveCollision(
		_newPositions,
		_newVelocities);
}

const std::shared_ptr<ParticleSystemData> ParticleSystemSolver::particleSystemData() const
{
	return _particleSystemData;
}

const ColliderPtr & ParticleSystemSolver::collider() const
{
	return _collider;
}

void ParticleSystemSolver::setCollider(const ColliderPtr & newCollider)
{
	_collider = newCollider;
}

const ParticleEmitterPtr & ParticleSystemSolver::emitter() const
{
	return _emitter;
}

void ParticleSystemSolver::beginAdvanceTimeStep(double timeIntervalInSeconds)
{
	_particleSystemData->forces().clear();
	_particleSystemData->forces().resize(_particleSystemData->numberOfParticles());
	
	updateCollider(timeIntervalInSeconds);

	updateEmitter(timeIntervalInSeconds);

	size_t n = _particleSystemData->numberOfParticles();
	_newPositions.clear();
	_newPositions.resize(n);
	_newVelocities.clear();
	_newVelocities.resize(n);
	
	onBeginAdvanceTimeStep(timeIntervalInSeconds);

}

void ParticleSystemSolver::endAdvanceTimeStep(double timeIntervalInSeconds)
{
	size_t n = _particleSystemData->numberOfParticles();

	for (size_t i = 0; i < n; i++)
	{
		_particleSystemData->positions()[i] = _newPositions[i];
		_particleSystemData->velocities()[i] = _newVelocities[i];
	}
	onEndAdvanceTimeStep(timeIntervalInSeconds);
}

void ParticleSystemSolver::onBeginAdvanceTimeStep(double timeStepInSeconds)
{
}

void ParticleSystemSolver::onEndAdvanceTimeStep(double timeStepInSeconds)
{
}

void ParticleSystemSolver::onInitialise()
{
	updateCollider(0.0);
	updateEmitter(0.0);
}

void ParticleSystemSolver::timeIntegration(double timeIntervalInSeconds)
{
	size_t n = _particleSystemData->numberOfParticles();
	auto forces = _particleSystemData->forces();
	const double mass = _particleSystemData->mass();

	for (size_t i = 0; i < n; i++)
	{
		Vector3 force = Vector3();
		if (i < forces.size())
		{
			force = forces[i];
		}

		_newVelocities[i] = _particleSystemData->velocities()[i].vectorAdd(((force.scalarDivide(mass)).scalarMultiply(timeIntervalInSeconds)));

		_newPositions[i] = _particleSystemData->positions()[i].vectorAdd((_newVelocities[i].scalarMultiply(timeIntervalInSeconds)));
	}
}

void ParticleSystemSolver::updateCollider(double timeStepInSeconds)
{
	if (_collider != nullptr)
	{
		_collider->update(currentTimeInSeconds(), timeStepInSeconds);
	}
}

void ParticleSystemSolver::updateEmitter(double timeStepInSeconds)
{
	if (_emitter != nullptr) 
	{
		_emitter->update(currentTimeInSeconds(), timeStepInSeconds);
	}
}

void ParticleSystemSolver::resolveCollision(
	std::vector<Vector3>& newPositions,
	std::vector<Vector3>& newVelocities)
{
	if (_collider != nullptr)
	{
		size_t numberOfParticles = _particleSystemData->numberOfParticles();
		const double radius = _particleSystemData->radius();

		for (size_t i = 0; i < numberOfParticles; i++)
		{
			_collider->resolveCollision(
				radius,
				_restitutionCoefficient,
				&newPositions[i],
				&newVelocities[i]);
		}
	}
}

void ParticleSystemSolver::setParticleSystemData(const ParticleSystemDataPtr & newParticles)
{
	_particleSystemData = newParticles;
}
