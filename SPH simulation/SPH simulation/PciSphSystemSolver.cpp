#include "PciSphSystemSolver.h"



PciSphSystemSolver::PciSphSystemSolver()
{
}


PciSphSystemSolver::PciSphSystemSolver(double targetDensity, double targetSpacing, double relativeKernelRadius)
	: SphSystemSolver(targetDensity, targetSpacing, relativeKernelRadius)
{

}

PciSphSystemSolver::~PciSphSystemSolver()
{
}

void PciSphSystemSolver::accumulatePressureForce(double timeIntervalInSeconds)
{
	auto particles = sphSystemData();
	const size_t numberOfParticles = particles->numberOfParticles();
	const double targetDensity = particles->targetDensity();
	const double mass = particles->mass();

	const double delta = computeDelta(timeIntervalInSeconds);
	std::vector<double> ds(numberOfParticles, 0.0);
	SphStdKernel kernel(particles->kernelRadius());


	//init buffers
	for (size_t i = 0; i < numberOfParticles; i++)
	{
		if (i >= particles->pressures().size())
		{
			sphSystemData()->pressures().push_back(0.0);
		}
		else
		{
			sphSystemData()->pressures()[i] = 0.0;
		}
		_pressureForces[i] = Vector3(0,0,0);
		_densityErrors[i] = 0.0;
		ds[i] = particles->densities()[i];
	}

	for (unsigned int k = 0; k < _maxNumberOfIterations; ++k)
	{
		//predict vel and pos
		for (size_t i = 0; i < numberOfParticles; i++)
		{
			Vector3 force = Vector3();
			if (i < particles->forces().size())
			{
				force = particles->forces()[i];
			}
			_tempVelocities[i] = particles->velocities()[i].vectorAdd((force.vectorAdd(_pressureForces[i])).scalarMultiply(timeIntervalInSeconds / mass));
			_tempPositions[i] = particles->positions()[i].vectorAdd(_tempVelocities[i].scalarMultiply(timeIntervalInSeconds));
		}
		//resolve collisions
		resolveCollision(
			_tempPositions,
			_tempVelocities);

		//compute pressure from density error
		for (size_t i = 0; i < numberOfParticles; i++)
		{
			double weightSum = 0.0;
			const auto& neighbours = particles->neighborLists()[i];

			for (size_t j : neighbours)
			{
				double dist = _tempPositions[j].distanceTo(_tempPositions[i]);
				weightSum += kernel(dist);
			}
			weightSum += kernel(0);

			double density = mass * weightSum;
			double densityError = (density - targetDensity);
			double pressure = delta * densityError;

			if (pressure < 0.0)
			{
				pressure *= negativePressureScale();
				densityError *= negativePressureScale();
			}
			else
			{
				pressure = pressure;
			}
			sphSystemData()->pressures()[i] += pressure;
			ds[i] = density;
			_densityErrors[i] = densityError;
		}

		//compute pressure gradient force
		for (size_t i = 0; i < _pressureForces.size(); i++)
		{
			_pressureForces[i] = Vector3();
		}
		SphSystemSolver::accumulatePressureForce(particles->positions(), ds, particles->pressures(), _pressureForces);

		//compute max density error
		double maxDensityError = 0.0;
		for (size_t i = 0; i < numberOfParticles; ++i)
		{
			maxDensityError = std::max(std::abs(maxDensityError), std::abs(_densityErrors[i]));
		}
		double densityErrorRatio = maxDensityError / targetDensity;

		if (std::fabs(densityErrorRatio) < _maxDensityErrorRatio)
		{
			break;
		}
	}

		//accumulate pressure force
		for (size_t i = 0; i < numberOfParticles; i++)
		{
			sphSystemData()->forces()[i] = sphSystemData()->forces()[i].vectorAdd(_pressureForces[i]);
		}
	
}

void PciSphSystemSolver::onBeginAdvanceTimeStep(double timeStepInSeconds)
{
	SphSystemSolver::onBeginAdvanceTimeStep(timeStepInSeconds);

	// Allocate temp buffers
	size_t numberOfParticles = particleSystemData()->numberOfParticles();
	_tempPositions.clear();
	_tempPositions.resize(numberOfParticles);
	_tempVelocities.clear();
	_tempVelocities.resize(numberOfParticles);
	_pressureForces.clear();
	_pressureForces.resize(numberOfParticles);
	_densityErrors.clear();
	_densityErrors.resize(numberOfParticles);
}

double PciSphSystemSolver::computeDelta(double timeStepInSeconds)
{
	auto particles = sphSystemData();
	const double kernelRadius = particles->kernelRadius();

	std::vector<Vector3> points;
	BccLatticePointGenerator pointsGenerator;
	Vector3 origin;
	BoundingBox sampleBound(origin, origin);
	sampleBound.expand(1.5*kernelRadius);

	pointsGenerator.generate(sampleBound, particles->targetSpacing(), &points);

	SphSpikyKernel kernel(kernelRadius);

	double denom = 0;
	Vector3 denom1;
	double denom2 = 0;

	for (size_t i = 0; i < points.size(); ++i) 
	{
		Vector3& point = points[i];
		double distanceSquared = point.lengthSquared();

		if (distanceSquared < kernelRadius * kernelRadius)
		{
			double distance = std::sqrt(distanceSquared);
			Vector3 direction =
				(distance > 0.0) ? (point.scalarDivide(distance)) : Vector3();

			// grad(Wij)
			Vector3 gradWij = kernel.gradient(distance, direction);
			denom1 = denom1.vectorAdd(gradWij);
			denom2 += gradWij.dot(gradWij);
		}
	}

	denom += -denom1.dot(denom1) - denom2;

	return (std::fabs(denom) > 0.0) ?
		-1 / (computeBeta(timeStepInSeconds) * denom) : 0;
}

double PciSphSystemSolver::computeBeta(double timeStepInSeconds)
{
	auto particles = sphSystemData();
	return 2.0 * (particles->mass() * timeStepInSeconds
		/ particles->targetDensity()) * (particles->mass() * timeStepInSeconds
			/ particles->targetDensity());
}

PciSphSystemSolver PciSphSystemSolver::Builder::build() const
{
	return PciSphSystemSolver(_targetDensity, _targetSpacing, _relativeKernelRadius);
}

PciSphSystemSolverPtr PciSphSystemSolver::Builder::makeShared() const
{
	return std::shared_ptr<PciSphSystemSolver>(
		new PciSphSystemSolver(
			_targetDensity, 
			_targetSpacing, 
			_relativeKernelRadius), 
		[](PciSphSystemSolver* obj) 
	{delete obj; });
}
