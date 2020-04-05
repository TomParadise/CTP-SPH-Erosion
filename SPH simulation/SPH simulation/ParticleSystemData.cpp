#include "ParticleSystemData.h"
#include <memory>

static const size_t kDefaultHashGridResolution = 64;

ParticleSystemData::ParticleSystemData() : ParticleSystemData(0){}

ParticleSystemData::ParticleSystemData(size_t numberOfParticles)
{	
	_neighbourSearcher = std::make_shared<PointParallelHashGridSearcher>(
		kDefaultHashGridResolution,
		kDefaultHashGridResolution,
		kDefaultHashGridResolution,
		2.0*_radius);

	resize(numberOfParticles);
}

ParticleSystemData::~ParticleSystemData()
{
}

void ParticleSystemData::resize(size_t newSize)
{
	_numberOfParticles = newSize;
	_forces.resize(newSize);
}

size_t ParticleSystemData::numberOfParticles() const
{
	return _numberOfParticles;
}

std::vector<Vector3>& ParticleSystemData::positions()
{
	return _positions;
}

std::vector<Vector3>& ParticleSystemData::velocities()
{
	return _velocities;
}

std::vector<Vector3>& ParticleSystemData::forces()
{
	return _forces;
}

void ParticleSystemData::setDensities(std::vector<double> densities)
{
	_densities = densities;
}

void ParticleSystemData::setPressures(std::vector<double> pressures)
{
	_pressures = pressures;
}

double ParticleSystemData::mass()
{
	return _mass;
}

void ParticleSystemData::setMass(double newMass)
{
	_mass = newMass;
}

double ParticleSystemData::radius()
{
	return _radius;
}

void ParticleSystemData::setRadius(double newRadius)
{
	_radius = newRadius;
}

void ParticleSystemData::addParticle(
	const Vector3 & newPos, 
	const Vector3 & newVel, 
	const Vector3 & newForce)
{
	std::vector<Vector3> newPositions = { newPos };
	std::vector<Vector3> newVelocities = { newVel };
	std::vector<Vector3> newForces = { newForce };

	addParticles(
		newPositions,
		newVelocities,
		newForces);
}

void ParticleSystemData::addParticles(
	const std::vector<Vector3>& newPositions,
	const std::vector<Vector3>& newVelocities,
	const std::vector<Vector3>& newForces)
{
	if (newVelocities.size() > 0 &&
		newVelocities.size() != newPositions.size() ||
		newForces.size() > 0 &&
		newForces.size() != newPositions.size())
	{
		return;
	}

	size_t oldNumberOfParticles = numberOfParticles();
	size_t newNumberOfParticles = oldNumberOfParticles + newPositions.size();

	resize(newNumberOfParticles);

	for (size_t i = 0; i < newPositions.size(); i++)
	{		
		if (i+oldNumberOfParticles >= _positions.size())
		{
			_positions.push_back(newPositions.at(i));
		}
		else
		{
			_positions.at(i + oldNumberOfParticles) = newPositions.at(i);
		}
	}
	if (newVelocities.size() > 0)
	{
		for (size_t i = 0; i < newPositions.size(); i++)
		{
			if (i + oldNumberOfParticles >= _velocities.size())
			{
				_velocities.push_back(newVelocities.at(i));
			}
			else
			{
				_velocities.at(i + oldNumberOfParticles) = newVelocities.at(i);
			}
		}
	}
	if (newForces.size() > 0)
	{
		for (size_t i = 0; i < newPositions.size(); i++)
		{
			if (i + oldNumberOfParticles >= _forces.size())
			{
				_forces.push_back(newForces.at(i));
			}
			else
			{
				_forces.at(i + oldNumberOfParticles) = newForces.at(i);
			}
		}
	}
}

void ParticleSystemData::set(const ParticleSystemData & other)
{
	_radius = other._radius;
	_mass = other._mass;
	_numberOfParticles = other._numberOfParticles;
}

void ParticleSystemData::buildNeighbourSearcher(double maxSearchRadius)
{
	_neighbourSearcher = std::make_shared<PointParallelHashGridSearcher>(
		kDefaultHashGridResolution,
		kDefaultHashGridResolution,
		kDefaultHashGridResolution,
		2.0*maxSearchRadius);
	
	_neighbourSearcher->build(_positions);
}

void ParticleSystemData::buildNeighbourLists(double maxSearchRadius)
{
	_neighbourLists.clear();
	_neighbourLists.resize(numberOfParticles());

	auto points = _positions;

	for (size_t i = 0; i < numberOfParticles(); ++i)
	{
		Vector3 origin = points[i];
		_neighbourLists[i].clear();

		_neighbourSearcher->forEachNearbyPoint(origin,
			maxSearchRadius,
			[&](size_t j, const Vector3&)
		{
			if (i != j)
			{
				_neighbourLists[i].push_back(j);
			}
		});
	}
}

const std::vector<std::vector<size_t>>& ParticleSystemData::neighborLists() const
{
	return _neighbourLists;
}

PointNeighbourSearcherPtr ParticleSystemData::neighborSearcher()
{
	return _neighbourSearcher;
}

double ParticleSystemData::targetDensity() const
{
	return _targetDensity;
}

size_t ParticleSystemData::addScalarData(double initialVal)
{
	size_t attrIdx = _scalarDataList.size();
	_scalarDataList.emplace_back(numberOfParticles(), initialVal);
	return attrIdx;
}

size_t ParticleSystemData::addVectorData(const Vector3 & initialVal)
{
	size_t attrIdx = _vectorDataList.size();
	_vectorDataList.emplace_back(numberOfParticles(), initialVal);
	return attrIdx;
}

ParticleSystemData::doubleArray & ParticleSystemData::scalarDataAt(size_t idx)
{
	return _scalarDataList.at(idx);
}
ParticleSystemData::vectorArray & ParticleSystemData::vectorDataAt(size_t idx)
{
	return _vectorDataList.at(idx);
}
