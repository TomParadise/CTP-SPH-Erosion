#include "SphSystemSolver.h"

#include <memory>
#include <cmath>
#include <algorithm>

SphSystemSolver::SphSystemSolver()
{
}


SphSystemSolver::SphSystemSolver(double targetDensity, double targetSpacing, double relativeKernelRadius)
{
	auto sphParticles = std::make_shared<SphSystemData>();
	setParticleSystemData(sphParticles);
	sphParticles->setTargetDensity(targetDensity);
	sphParticles->setTargetSpacing(targetSpacing);
	sphParticles->setRelativeKernelRadius(relativeKernelRadius);	
}

SphSystemSolver::~SphSystemSolver()
{
}

double SphSystemSolver::negativePressureScale() const
{
	return _negativePressureScale;
}

void SphSystemSolver::setNegativePressureScale(double newNegativePressureScale)
{
	if(newNegativePressureScale < 0.0)
	{
		newNegativePressureScale = 0.0;
	}
	else if (newNegativePressureScale > 1.0)
	{
		newNegativePressureScale = 1.0;
	}
	_negativePressureScale = newNegativePressureScale;
}

void SphSystemSolver::setPseudoViscosityCoefficient(double newPseudoViscosityCoefficient)
{
	_pseudoViscosityCoefficient
		= std::max(newPseudoViscosityCoefficient, 0.0);
}

void SphSystemSolver::setTimeStepLimitScale(double newScale)
{
	_timeStepLimitScale = std::max(newScale, 0.0);
}

unsigned int SphSystemSolver::numberOfSubTimeSteps(double timeIntervalInSeconds) const
{
	auto particles = sphSystemData();
	size_t numberOfParticles = particles->numberOfParticles();

	const double kernelRadius = particles->kernelRadius();
	const double mass = particles->mass();

	double maxForceMagnitude = 0.0;

	for (size_t i = 0; i < numberOfParticles; ++i) 
	{
		maxForceMagnitude = std::max(maxForceMagnitude, particles->forces()[i].length());
	}

	double timeStepLimitBySpeed
		= 0.4 * kernelRadius / _speedOfSound;
	double timeStepLimitByForce
		= 0.25
		* std::sqrt(kernelRadius * mass / maxForceMagnitude);

	double desiredTimeStep
		= _timeStepLimitScale
		* std::min(timeStepLimitBySpeed, timeStepLimitByForce);

	return static_cast<unsigned int>(
		std::ceil(timeIntervalInSeconds / desiredTimeStep));
}

void SphSystemSolver::accumulateForces(double timeStepInSeconds)
{
	accumulateNonPressureForces(timeStepInSeconds);
	accumulatePressureForce(timeStepInSeconds);
}

void SphSystemSolver::onBeginAdvanceTimeStep(double timeStepInSeconds)
{
	sphSystemData()->buildNeighbourSearcher();
	sphSystemData()->buildNeighbourLists();
	sphSystemData()->updateDensities();
}

void SphSystemSolver::onEndAdvanceTimeStep(double timeStepInSeconds)
{
	if (_pseudoViscosityCoefficient != 0)
	{
		computePseudoViscosity(timeStepInSeconds);
	}
}

void SphSystemSolver::accumulateNonPressureForces(double timeStepInSeconds)
{
	ParticleSystemSolver::accumulateForces(timeStepInSeconds);
	accumulateViscosityForce();
}

void SphSystemSolver::accumulatePressureForce(double timeStepInSeconds)
{
	computePressure();
	accumulatePressureForce(sphSystemData()->positions(), sphSystemData()->densities(), sphSystemData()->pressures(), sphSystemData()->forces());
}

void SphSystemSolver::accumulatePressureForce(const std::vector<Vector3>& positions, const std::vector<double>& densities, const std::vector<double>& pressures, std::vector<Vector3>& pressureForces)
{
	auto particles = sphSystemData();
	size_t numberOfParticles = particles->numberOfParticles();

	const double massSquared = particles->mass()*particles->mass();
	const SphSpikyKernel kernel(particles->kernelRadius());

	for (size_t i = 0; i < numberOfParticles; i++)
	{
		const auto& neighbours = particles->neighborLists()[i];
		for (size_t j : neighbours)
		{
			Vector3 vec = positions[i];
			double dist = vec.distanceTo(positions[j]);

			if (dist > 0.0)
			{
				Vector3 dir = positions[j];
				dir -= vec;
				dir /= dist;
				pressureForces[i] -= (kernel.gradient(dist, dir) *
						massSquared *
						(pressures[i] / (densities[i]*densities[i]) +
							pressures[j] / (densities[j]*densities[j])));
			}
		}
	}
}

void SphSystemSolver::computePressure()
{
	auto particles = sphSystemData();
	size_t numberOfParticles = particles->numberOfParticles();

	const double targetDensity = particles->targetDensity();
	const double eosScale = targetDensity * _speedOfSound * _speedOfSound;
	for(size_t i = 0; i<numberOfParticles; i++)
	{
		if (i >= sphSystemData()->pressures().size())
		{
			sphSystemData()->pressures().push_back(computePressureFromEos(sphSystemData()->densities()[i], targetDensity, eosScale, _eosExponent, negativePressureScale()));
		}
		else
		{
			sphSystemData()->pressures().at(i) = computePressureFromEos(sphSystemData()->densities()[i], targetDensity, eosScale, _eosExponent, negativePressureScale());
		}
	}
}

void SphSystemSolver::accumulateViscosityForce()
{
	auto particles = sphSystemData();
	size_t numberOfParticles = particles->numberOfParticles();
	auto x = particles->positions();
	auto v = particles->velocities();
	auto d = particles->densities();

	const double massSquared = particles->mass() * particles->mass();
	const SphSpikyKernel kernel(particles->kernelRadius());

	for (size_t i = 0; i < numberOfParticles; ++i)
	{
		const auto& neighbours = particles->neighborLists()[i];
		for (size_t j : neighbours)
		{
			double dist = x[i].distanceTo(x[j]);			

			Vector3 add = (v[j]-v[i])/d[j];
			add *= _viscosityCoefficient * massSquared * kernel.secondDerivative(dist);

			sphSystemData()->forces()[i] += add;			
		}
	}
}

void SphSystemSolver::computePseudoViscosity(double timeStepInSeconds)
{
	auto particles = sphSystemData();
	size_t numberOfParticles = particles->numberOfParticles();
	auto x = particles->positions();
	auto v = particles->velocities();
	auto d = particles->densities();

	const double mass = particles->mass();
	const SphSpikyKernel kernel(particles->kernelRadius());

	std::vector<Vector3> smoothedVelocities(numberOfParticles);

	for (size_t i = 0; i < numberOfParticles; i++)
	{
		double weightSum = 0.0;
		Vector3 smoothedVelocity;

		const auto& neighbours = particles->neighborLists().at(i);
		for (size_t j : neighbours)
		{
			double dist = x[i].distanceTo(x[j]);
			double wj = mass / d[j] * kernel(dist);

			weightSum += wj;
			smoothedVelocity += v[j] * wj;
		}

		double wi = mass / d[i];
		weightSum += wi;
		smoothedVelocity += v[i] * wi;

		if (weightSum > 0.0)
		{
			smoothedVelocity /= weightSum;
		}

		smoothedVelocities.at(i) = smoothedVelocity;
	}

	double factor = timeStepInSeconds * _pseudoViscosityCoefficient;
	if (factor < 0.0) factor = 0.0;
	else if (factor > 1.0) factor = 1.0;

	for (size_t i = 0; i < numberOfParticles; i++)
	{
		particles->velocities()[i] += (smoothedVelocities[i] - v[i]) * factor;
	}
	
}

SphSystemDataPtr SphSystemSolver::sphSystemData() const
{
	return std::dynamic_pointer_cast<SphSystemData>(particleSystemData());
}

double SphSystemSolver::computePressureFromEos(double density, double targetDensity, double eosScale, double eosExponent, double negativePressureScale)
{
	double p = eosScale / eosExponent * (std::pow((density / targetDensity), eosExponent) - 1.0);

	if (p < 0)
	{
		p *= negativePressureScale;
	}
	return p;
}
