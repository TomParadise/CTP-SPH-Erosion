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
	const double mass = _particleSystemData->mass();

	for (size_t i = 0; i < n; i++)
	{
		//gravity and drag
		_particleSystemData->forces()[i] += (_gravity * mass) +
						(_particleSystemData->velocities()[i] *
							-_dragCoefficient);
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
	onEndAdvanceTimeStep(timeIntervalInSeconds);
	size_t n = _particleSystemData->numberOfParticles();
	double nsqrt = std::sqrt(n);
	for (size_t i = 0; i < n; i++)
	{
		if (_newPositions[i].z <= 0)
		{
			_newPositions[i].z = 0;
		}
		if (_newPositions[i].x <= 0)
		{
			_newPositions[i].x = 0;
		}
		if (_collider->surface()->closestDistance(_newPositions[i]) <= _particleSystemData->radius() + 0.03)
		{
			double deltaHeight = _newPositions[i].y - _particleSystemData->positions()[i].y;
			double speed = _newVelocities[i].length()/nsqrt*25;

			// Calculate the droplet's sediment capacity 
			// (higher when moving fast down a slope and contains lots of water)
			double sedimentCapacity = std::max(-deltaHeight * speed * _particleSystemData->water()[i] * 4, 0.01 / nsqrt) * _particleSystemData->scalarDataAt(0)[i] / 850;

			// If carrying more sediment than capacity, or if flowing uphill:
			if (_particleSystemData->sediment()[i] > sedimentCapacity || deltaHeight > 0 && _particleSystemData->sediment()[i] > 0)
			{
				// If moving uphill (deltaHeight > 0) try fill up to the current height 
				// otherwise deposit a fraction of the excess sediment
				double amountToDeposit =
					((deltaHeight > 0) ? std::min(deltaHeight, _particleSystemData->sediment()[i]) :
					(_particleSystemData->sediment()[i] - sedimentCapacity)) * 0.3f;

				_particleSystemData->sediment()[i] -= amountToDeposit;

				_collider->surface()->depositToNode(_newPositions[i], amountToDeposit);
			}
			else
			{
				// Erode a fraction of the droplet's current carry capacity.
				// Clamp the erosion to the change in height so that it doesn't 
				// dig a hole in the terrain behind the droplet
				double amountToErode = std::min((sedimentCapacity - _particleSystemData->sediment()[i]) *
					0.3f,
					-deltaHeight);

				_particleSystemData->sediment()[i] += _collider->surface()->erodeNode(_newPositions[i], amountToErode);
			}
			_particleSystemData->water()[i] *= (1 - 0.05);
			if (_particleSystemData->water()[i] <= 0)
			{
				_newPositions[i] = _emitter->getRandomSpawnPos();
				_newVelocities[i] = Vector3();
				_particleSystemData->water()[i] = 1 / nsqrt;
				_particleSystemData->sediment()[i] = 0;
			}
		}
	}

	for (size_t i = 0; i < n; i++)
	{
		_particleSystemData->positions()[i] = _newPositions[i];
		_particleSystemData->velocities()[i] = _newVelocities[i];
	}
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
	const double mass = _particleSystemData->mass();

	for (size_t i = 0; i < n; i++)
	{

		_newVelocities[i] = _particleSystemData->velocities()[i]+(((_particleSystemData->forces()[i]/(mass))*(timeIntervalInSeconds)));

		_newPositions[i] = _particleSystemData->positions()[i]+((_newVelocities[i]*(timeIntervalInSeconds)));
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
			//respawn if not inside the bounds of the heightmap
			if (!_collider->surface()->boundingBox().contains(newPositions[i]) || newPositions[i].z <= 0 || newPositions[i].x <= 0)
			{
				newPositions[i] = _emitter->getRandomSpawnPos();
				newVelocities[i] = Vector3(); 
				_particleSystemData->water()[i] = 1;
				_particleSystemData->sediment()[i] = 0;

			}
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
