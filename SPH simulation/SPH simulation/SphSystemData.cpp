#include "SphSystemData.h"

#include <algorithm>

SphSystemData::SphSystemData() : SphSystemData(0) {}

SphSystemData::SphSystemData(size_t numberOfParticles)
	: ParticleSystemData(numberOfParticles) 
{
	_densityIdx = addScalarData();
	_pressureIdx = addScalarData();

	setTargetSpacing(_targetSpacing);
	pressures().resize(numberOfParticles);
}

SphSystemData::SphSystemData(const SphSystemData & other)
{
	set(other);
}

SphSystemData::~SphSystemData()
{
}

std::vector<double>& SphSystemData::densities()
{
	return scalarDataAt(_densityIdx);
}

std::vector<double>& SphSystemData::pressures()
{
	return scalarDataAt(_pressureIdx);
}

void SphSystemData::setTargetDensity(double targetDensity)
{
	_targetDensity = targetDensity;

	computeMass();
}

double SphSystemData::targetDensity() const
{
	return _targetDensity;
}

void SphSystemData::setTargetSpacing(double spacing)
{
	ParticleSystemData::setRadius(spacing);

	_targetSpacing = spacing;
	_kernelRadius = _kernelRadiusOverTargetSpacing * _targetSpacing;

	computeMass();
}

double SphSystemData::targetSpacing() const
{
	return _targetSpacing;
}

void SphSystemData::setRelativeKernelRadius(double relativeRadius)
{
	_kernelRadiusOverTargetSpacing = relativeRadius;
	_kernelRadius = _kernelRadiusOverTargetSpacing * _targetSpacing;

	computeMass();
}

void SphSystemData::setKernelRadius(double kernelRadius)
{
	_kernelRadius = kernelRadius;
	_targetSpacing = kernelRadius / _kernelRadiusOverTargetSpacing;

	computeMass();
}

void SphSystemData::updateDensities()
{
	for (size_t i = 0; i < numberOfParticles(); i++)
	{
		double sum = sumOfKernelNearby(positions()[i]);
		if (i >= densities().size())
		{
		    densities().push_back(mass() * sum);
		}
		else
		{
			densities()[i] = mass() * sum;
		}
	}
}

double SphSystemData::sumOfKernelNearby(Vector3 & pos)
{
	double sum = 0.0;

	SphStdKernel kernel(_kernelRadius);
	neighborSearcher()->forEachNearbyPoint(
		pos,
		_kernelRadius,
		[&](size_t, const Vector3& neighbourPosition)
	{
		double dist = pos.distanceTo(neighbourPosition);
		sum += kernel(dist);
	});
	return sum;
}

Vector3 SphSystemData::interpolate(
	Vector3& origin,
	std::vector<Vector3>& values)
{
	Vector3 sum;
	auto d = densities();

	SphStdKernel kernel(_kernelRadius);
	double m = mass();

	neighborSearcher()->forEachNearbyPoint(
		origin,
		_kernelRadius,
		[&](size_t i, const Vector3& neighbourPosition)
	{
		double dist = origin.distanceTo(neighbourPosition);
		double weight = m / d[i] * kernel(dist);
		sum += values[i] * (weight);
	});
	return sum;
}

double SphSystemData::interpolate(
	Vector3& origin)
{
	double sum;
	auto d = densities();

	SphStdKernel kernel(_kernelRadius);
	double m = mass();

	neighborSearcher()->forEachNearbyPoint(
		origin,
		_kernelRadius,
		[&](size_t i, const Vector3& neighbourPosition)
	{
		double dist = origin.distanceTo(neighbourPosition);
		double weight = m * kernel(dist);
		sum += weight;
	});
	return sum;
}

Vector3 SphSystemData::gradientAt(size_t i, const std::vector<double>& values)
{
	Vector3 sum;
	auto p = positions();
	auto d = densities();
	const auto& neighbours = neighborLists()[i];
	Vector3 origin = p.at(i);
	SphSpikyKernel kernel(_kernelRadius);

	for (size_t j : neighbours)
	{
		Vector3 neighbourPosition = p.at(j);
		double dist = origin.distanceTo(neighbourPosition);
		if (dist > 0.0)
		{
			Vector3 dir = (neighbourPosition-origin) / dist;
			sum += kernel.gradient(dist, dir) * (d[i] * mass() *
				(values[i] / (d[i] * d[i]) + values[j] / (d[j] * d[j])));
		}
	}
	return sum;
}

double SphSystemData::laplacianAt(size_t i, const std::vector<double>& values)
{
	double sum = 0.0;
	auto p = positions();
	auto d = densities();
	const auto& neightbours = neighborLists()[i];
	Vector3& origin = p.at(i);
	SphSpikyKernel kernel(_kernelRadius);

	for (size_t j : neightbours)
	{
		Vector3 neighbourPosition = p.at(j);
		double dist = origin.distanceTo(neighbourPosition);
		sum += mass() * (values[j] - values[i]) / d[j] * kernel.secondDerivative(dist);
	}
	return sum;
}

void SphSystemData::buildNeighbourSearcher()
{
	ParticleSystemData::buildNeighbourSearcher(_kernelRadius);
}

void SphSystemData::buildNeighbourLists()
{
	ParticleSystemData::buildNeighbourLists(_kernelRadius);
}

double SphSystemData::kernelRadius() const
{
	return _kernelRadius;
}

void SphSystemData::computeMass()
{
	std::vector<Vector3> points;
	BccLatticePointGenerator pointsGenerator;
	BoundingBox sampleBound(
		Vector3(-1.5 * _kernelRadius, -1.5 * _kernelRadius,
			-1.5 * _kernelRadius),
		Vector3(1.5 * _kernelRadius, 1.5 * _kernelRadius,
			1.5 * _kernelRadius));

	pointsGenerator.generate(sampleBound, _targetSpacing, &points);

	double maxNumberDensity = 0.0;
	SphStdKernel kernel(_kernelRadius);

	for (size_t i = 0; i < points.size(); ++i) 
	{
		const Vector3& point = points[i];
		double sum = 0.0;

		for (size_t j = 0; j < points.size(); ++j)
		{
			Vector3& neighbourPoint = points[j];
			sum += kernel(neighbourPoint.distanceTo(point));
		}

		maxNumberDensity = std::max(maxNumberDensity, sum);
	}

	double newMass = _targetDensity / maxNumberDensity;

	ParticleSystemData::setMass(newMass);
}
