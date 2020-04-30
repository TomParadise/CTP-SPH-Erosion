#pragma once
#ifndef INCLUDE_SPH_SYSTEM_DATA_H_
#define INCLUDE_SPH_SYSTEM_DATA_H_

#include <memory>

#include "ParticleSystemData.h"
#include "Vector3.h"
#include "SphStdKernel.h"
#include "SphSpikyKernel.h"
#include "BoundingBox.h"
#include "BccLatticePointGenerator.h"

class SphSystemData : public ParticleSystemData
{
public:
	//! Constructs empty SPH system.
	SphSystemData();

	//! Constructs SPH system data with given number of particles.
	explicit SphSystemData(size_t numberOfParticles);

	//! Copy constructor.
	SphSystemData(const SphSystemData& other);

	virtual ~SphSystemData();

	ParticleSystemData::doubleArray& densities();

	ParticleSystemData::doubleArray& pressures();


	//! Sets the target density of this particle system.
	void setTargetDensity(double targetDensity);

	//! Returns the target density of this particle system.
	double targetDensity() const;

	//!
	//! \brief Sets the target particle spacing in meters.
	//!
	//! Once this function is called, hash grid and density should be
	//! updated using updateHashGrid() and updateDensities).
	//!
	void setTargetSpacing(double spacing);

	//! Returns the target particle spacing in meters.
	double targetSpacing() const;

	//!
	//! \brief Sets the relative kernel radius.
	//!
	//! Sets the relative kernel radius compared to the target particle
	//! spacing (i.e. kernel radius / target spacing).
	//! Once this function is called, hash grid and density should
	//! be updated using updateHashGrid() and updateDensities).
	//!
	void setRelativeKernelRadius(double relativeRadius);

	//!
	//! \brief Sets the absolute kernel radius.
	//!
	//! Sets the absolute kernel radius compared to the target particle
	//! spacing (i.e. relative kernel radius * target spacing).
	//! Once this function is called, hash grid and density should
	//! be updated using updateHashGrid() and updateDensities).
	//!
	void setKernelRadius(double kernelRadius);

	void updateDensities();

	double sumOfKernelNearby(Vector3& pos);

	Vector3 interpolate(
		Vector3& origin,
		std::vector<Vector3>& values);

	double interpolate(
		Vector3& origin);

	Vector3 gradientAt(size_t i, const std::vector<double>& values);
	double laplacianAt(size_t i, const std::vector<double>& values);

	//! Builds neighbor searcher with kernel radius.
	void buildNeighbourSearcher();

	//! Builds neighbor lists with kernel radius.
	void buildNeighbourLists();

	double kernelRadius() const;
private:
	double _kernelRadius;

	//! Target density of this particle system in kg/m^3.
	double _targetDensity = 1000.0;

	//! Target spacing of this particle system in meters.
	double _targetSpacing = 0.1;

	//! Relative radius of SPH kernel.
	//! SPH kernel radius divided by target spacing.
	double _kernelRadiusOverTargetSpacing = 1.8;

	size_t _pressureIdx;

	size_t _densityIdx;

	void computeMass();
};

//! Shared pointer for the SphSystemData3 type.
typedef std::shared_ptr<SphSystemData> SphSystemDataPtr;

#endif