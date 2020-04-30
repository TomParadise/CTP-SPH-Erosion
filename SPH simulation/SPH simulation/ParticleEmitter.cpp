#include "ParticleEmitter.h"



ParticleEmitter::ParticleEmitter()
{
}


ParticleEmitter::~ParticleEmitter()
{
}

void ParticleEmitter::update(double currentTimeInSeconds, double timeIntervalInSeconds)
{
	if (_onBeginUpdateCallback)
	{
		_onBeginUpdateCallback(this, currentTimeInSeconds, timeIntervalInSeconds);
	}
	onUpdate(currentTimeInSeconds, timeIntervalInSeconds);
}

const ParticleSystemDataPtr & ParticleEmitter::target() const
{
	return _particles;
}

void ParticleEmitter::setTarget(const ParticleSystemDataPtr & particles)
{
	_particles = particles;

	//onSetTarget(particles);
}

bool ParticleEmitter::isEnabled() const
{
	return _isEnabled;
}

void ParticleEmitter::setIsEnabled(bool enabled)
{
	_isEnabled = enabled;
}

void ParticleEmitter::setOnBeginUpdateCallback(const OnBeginUpdateCallback & callback)
{
	_onBeginUpdateCallback = callback;
}
