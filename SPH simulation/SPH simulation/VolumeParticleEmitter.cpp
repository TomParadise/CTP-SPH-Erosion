#include "VolumeParticleEmitter.h"
#include "SurfaceToImplicit.h"

#include <iostream>

VolumeParticleEmitter::VolumeParticleEmitter(const ImplicitSurfacePtr & implicitSurface, const BoundingBox & maxRegion, double spacing, const Vector3 & initialVel, const Vector3 & linearVel, const Vector3 & angularVel, size_t maxNumberOfParticles, double jitter, bool isOneShot, bool allowOverlapping, uint32_t seed)
	: _rng(seed),
	_implicitSurface(implicitSurface),
	_bounds(maxRegion),
	_spacing(spacing),
	_initialVel(initialVel),
	_linearVel(linearVel),
	_angularVel(angularVel),
	_maxNumberOfParticles(maxNumberOfParticles),
	_jitter(jitter),
	_isOneShot(isOneShot),
	_allowOverlapping(allowOverlapping) 
{
	_pointsGen = std::make_shared<BccLatticePointGenerator>();
}

void VolumeParticleEmitter::setPointGenerator(const PointGeneratorPtr & newPointsGen)
{
	_pointsGen = newPointsGen;
}

const ImplicitSurfacePtr & VolumeParticleEmitter::surface() const
{
	return _implicitSurface;
}

void VolumeParticleEmitter::setSurface(const ImplicitSurfacePtr & newSurface)
{
	_implicitSurface = newSurface;
}

const BoundingBox & VolumeParticleEmitter::maxRegion() const
{
	return _bounds;
}

void VolumeParticleEmitter::setMaxRegion(const BoundingBox & newBox)
{
	_bounds = newBox;
}

double VolumeParticleEmitter::jitter() const
{
	return _jitter;
}

void VolumeParticleEmitter::setJitter(double newJitter)
{
	if (newJitter < 0.0)
	{
		_jitter = 0.0;
	}
	else if (newJitter > 1.0)
	{
		_jitter = 1.0;
	}
	else
	{
		_jitter = newJitter;
	}
}

bool VolumeParticleEmitter::isOneShot() const
{
	return _isOneShot;
}

void VolumeParticleEmitter::setIsOneShot(bool newValue)
{
	_isOneShot = newValue;
}

bool VolumeParticleEmitter::allowOverlapping() const
{
	return _allowOverlapping;
}

void VolumeParticleEmitter::setAllowOverlapping(bool newValue)
{
	_allowOverlapping = newValue;
}

size_t VolumeParticleEmitter::maxNumberOfParticles() const
{
	return _maxNumberOfParticles;
}

void VolumeParticleEmitter::setMaxNumberOfParticles(size_t newMaxNumberOfParticles)
{
	_maxNumberOfParticles = newMaxNumberOfParticles;
}

double VolumeParticleEmitter::spacing() const
{
	return _spacing;
}

void VolumeParticleEmitter::setSpacing(double newSpacing)
{
	_spacing = newSpacing;
}

Vector3 VolumeParticleEmitter::initialVelocity() const
{
	return _initialVel;
}

void VolumeParticleEmitter::setInitialVelocity(const Vector3 & newInitialVel)
{
	_initialVel = newInitialVel;
}

Vector3 VolumeParticleEmitter::linearVelocity() const
{
	return _linearVel;
}

void VolumeParticleEmitter::setLinearVelocity(const Vector3 & newLinearVel)
{
	_linearVel = newLinearVel;
}

Vector3 VolumeParticleEmitter::angularVelocity() const
{
	return _angularVel;
}

void VolumeParticleEmitter::setAngularVelocity(const Vector3 & newAngularVel)
{
	_angularVel = newAngularVel;
}

VolumeParticleEmitter::Builder VolumeParticleEmitter::builder()
{
	return Builder();
}

void VolumeParticleEmitter::onUpdate(double currentTimeInSeconds, double timeIntervalInSeconds)
{
	auto particles = target();

	if (particles == nullptr)
	{
		return;
	}

	if (!isEnabled())
	{
		return;
	}

	std::vector<Vector3> newPositions;
	std::vector<Vector3> newVelocities;

	emit(particles, &newPositions, &newVelocities);

	particles->addParticles(newPositions, newVelocities);

	if (_isOneShot)
	{
		setIsEnabled(false);
	}
}

void VolumeParticleEmitter::emit(const ParticleSystemDataPtr & particles, std::vector<Vector3>* newPositions, std::vector<Vector3>* newVelocities)
{
	if (!_implicitSurface)
	{
		return;
	}

	BoundingBox region = _bounds;
	BoundingBox surfaceBBox = _implicitSurface->boundingBox();

	region.lowerCorner = Vector3(
		std::max(region.lowerCorner.x, surfaceBBox.lowerCorner.x), 
		std::max(region.lowerCorner.y, surfaceBBox.lowerCorner.y), 
		std::max(region.lowerCorner.z, surfaceBBox.lowerCorner.z));
	region.upperCorner = Vector3(
		std::min(region.upperCorner.x, surfaceBBox.upperCorner.x), 
		std::min(region.upperCorner.y, surfaceBBox.upperCorner.y), 
		std::min(region.upperCorner.z, surfaceBBox.upperCorner.z));


	// Reserving more space for jittering
	const double j = jitter();
	const double maxJitterDist = 0.5 * j * _spacing;
	size_t numNewParticles = 0;

	if (_allowOverlapping || _isOneShot)
	{
		_pointsGen->forEachPoint(region, _spacing, [&](const Vector3& point) 
		{
			Vector3 randomDir = uniformSampleSphere(random(), random());
			Vector3 offset = randomDir.scalarMultiply(maxJitterDist);
			Vector3 candidate = point;
			candidate = candidate.vectorAdd(offset);
			if (_implicitSurface->signedDistance(candidate) <= 0.0) 
			{
				if (_numberOfEmittedParticles < _maxNumberOfParticles) 
				{
					newPositions->push_back(candidate);
					++_numberOfEmittedParticles;
					++numNewParticles;
				}
				else
				{
					return false;
				}
			}

			return true;
		});
	}
	else
	{
		// Use serial hash grid searcher for continuous update.
		PointHashGridSearcher neighborSearcher(Vector3(64,64,64),
			2.0 * _spacing);
		if (!_allowOverlapping) 
		{
			neighborSearcher.build(particles->positions());
		}

		_pointsGen->forEachPoint(region, _spacing, [&](const Vector3& point) 
		{
			Vector3 randomDir = uniformSampleSphere(random(), random());
			Vector3 offset = randomDir.scalarMultiply(maxJitterDist);
			Vector3 candidate = point;
			candidate.vectorAdd(offset);
			if (_implicitSurface->isInside(candidate) &&
				(!_allowOverlapping &&
					!neighborSearcher.hasNearbyPoint(candidate, _spacing))) 
			{
				if (_numberOfEmittedParticles < _maxNumberOfParticles)
				{
					newPositions->push_back(candidate);
					neighborSearcher.add(candidate);
					++_numberOfEmittedParticles;
					++numNewParticles;
				}
				else {
					return false;
				}
			}

			return true;
		});
	}

	std::cout << "Number of newly generated particles: " << numNewParticles << "\n";
	std::cout << "Number of total generated particles: "
		<< _numberOfEmittedParticles << "\n";

	newVelocities->clear();
	newVelocities->resize(newPositions->size());
	for(size_t i = 0; i<newVelocities->size();i++)
	{
		(*newVelocities)[i] = velocityAt((*newPositions)[i]);
	}
}

double VolumeParticleEmitter::random()
{
	std::uniform_real_distribution<> d(0.0, 1.0);
	return d(_rng);
}

Vector3 VolumeParticleEmitter::uniformSampleSphere(float u1, float u2)
{
	float y = 1 - 2 * u1;
	float r = std::sqrt(std::max<float>(0, 1 - y * y));
	float phi = (float)(2 * 3.14159265358979323846264338327950288 * u2);
	float x = r * std::cos(phi);
	float z = r * std::sin(phi);
	return Vector3(x, y, z);
}

Vector3 VolumeParticleEmitter::velocityAt(Vector3 & point)
{
	Vector3 r = point.vectorSubtract(_implicitSurface->transform.translation());
	return _linearVel.vectorAdd(_angularVel.cross(r)).vectorAdd(_initialVel);
}

VolumeParticleEmitter::Builder & VolumeParticleEmitter::Builder::withImplicitSurface(const ImplicitSurfacePtr & implicitSurface)
{
	_implicitSurface = implicitSurface;
	if (!_isBoundSet) 
	{
		_bounds = _implicitSurface->boundingBox();
	}
	return *this;
}

VolumeParticleEmitter::Builder & VolumeParticleEmitter::Builder::withSurface(const SurfacePtr & surface)
{
	_implicitSurface = std::make_shared<SurfaceToImplicit>(surface);
	if (!_isBoundSet) 
	{
		_bounds = surface->boundingBox();
	}
	return *this;
}

VolumeParticleEmitter::Builder & VolumeParticleEmitter::Builder::withMaxRegion(const BoundingBox & bounds)
{
	_bounds = bounds;
	_isBoundSet = true;
	return *this;
}

VolumeParticleEmitter::Builder & VolumeParticleEmitter::Builder::withSpacing(double spacing)
{
	_spacing = spacing;
	return *this;
}

VolumeParticleEmitter::Builder & VolumeParticleEmitter::Builder::withInitialVelocity(const Vector3 & initialVel)
{
	_initialVel = initialVel;
	return *this;
}

VolumeParticleEmitter::Builder & VolumeParticleEmitter::Builder::withLinearVelocity(const Vector3 & linearVel)
{
	_linearVel = linearVel;
	return *this;
}

VolumeParticleEmitter::Builder & VolumeParticleEmitter::Builder::withAngularVelocity(const Vector3 & angularVel)
{
	_angularVel = angularVel;
	return *this;
}

VolumeParticleEmitter::Builder & VolumeParticleEmitter::Builder::withMaxNumberOfParticles(size_t maxNumberOfParticles)
{
	_maxNumberOfParticles = maxNumberOfParticles;
	return *this;
}

VolumeParticleEmitter::Builder & VolumeParticleEmitter::Builder::withJitter(double jitter)
{
	_jitter = jitter;
	return *this;
}

VolumeParticleEmitter::Builder & VolumeParticleEmitter::Builder::withIsOneShot(bool isOneShot)
{
	_isOneShot = isOneShot;
	return *this;
}

VolumeParticleEmitter::Builder & VolumeParticleEmitter::Builder::withAllowOverlapping(bool allowOverlapping)
{
	_allowOverlapping = allowOverlapping;
	return *this;
}

VolumeParticleEmitter::Builder & VolumeParticleEmitter::Builder::withRandomSeed(uint32_t seed)
{
	_seed = seed;
	return *this;
}

VolumeParticleEmitter VolumeParticleEmitter::Builder::build() const
{
	return VolumeParticleEmitter(_implicitSurface, _bounds, _spacing,
		_initialVel, _linearVel, _angularVel,
		_maxNumberOfParticles, _jitter, _isOneShot,
		_allowOverlapping, _seed);
}

VolumeParticleEmitterPtr VolumeParticleEmitter::Builder::makeShared() const
{
	return std::shared_ptr<VolumeParticleEmitter>(
		new VolumeParticleEmitter(_implicitSurface, _bounds, _spacing,
			_initialVel, _linearVel, _angularVel,
			_maxNumberOfParticles, _jitter, _isOneShot,
			_allowOverlapping),
		[](VolumeParticleEmitter* obj) { delete obj; });
}