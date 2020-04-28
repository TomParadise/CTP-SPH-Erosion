#pragma once
#ifndef INCLUDE_PARTICLE_SYSTEM_DATA_H_
#define INCLUDE_PARTICLE_SYSTEM_DATA_H_

#include <math.h>
#include <memory>
#include <vector>
#include "Vector3.h"
#include "PointHashGridSearcher.h"

class ParticleSystemData
{
public:
	typedef std::vector<Vector3> vectorArray;
	typedef std::vector<double> doubleArray;

	ParticleSystemData();
	explicit ParticleSystemData(size_t numberOfParticles);
	virtual ~ParticleSystemData();

	void resize(size_t newSize);
	size_t numberOfParticles() const;

	std::vector<Vector3>& positions();
	std::vector<Vector3>& velocities();
	std::vector<Vector3>& forces();
	void setDensities(std::vector<double> densities);
	void setPressures(std::vector<double> pressures);

	std::vector<double>& water();
	std::vector<double>& sediment();

	double mass();

	virtual void setMass(double newMass);

	double radius();

	virtual void setRadius(double newRadius);

	void addParticle(
		const Vector3& newPos,
		const Vector3& newVel = Vector3(),
		const Vector3& newForce = Vector3());
	void addParticles(
		const std::vector<Vector3>& newPositions,
		const std::vector<Vector3>& newVelocities = std::vector<Vector3>(),
		const std::vector<Vector3>& newForces = std::vector<Vector3>()
	);

	void set(const ParticleSystemData& other);

	void buildNeighbourSearcher(double maxSearchRadius);
	void buildNeighbourLists(double maxSearchRadius);
	
	const std::vector<std::vector<size_t>>& neighborLists() const;

	PointNeighbourSearcherPtr neighborSearcher();

	double targetDensity() const;

	//!
	//! \brief      Adds a scalar data layer and returns its index.
	//!
	//! This function adds a new scalar data layer to the system. It can be used
	//! for adding a scalar attribute, such as temperature, to the particles.
	//!
	//! \params[in] initialVal  Initial value of the new scalar data.
	//!
	size_t addScalarData(double initialVal = 0.0);

	//!
	//! \brief      Adds a vector data layer and returns its index.
	//!
	//! This function adds a new vector data layer to the system. It can be used
	//! for adding a vector attribute, such as vortex, to the particles.
	//!
	//! \params[in] initialVal  Initial value of the new vector data.
	//!
	size_t addVectorData(const Vector3& initialVal = Vector3());

	//! Returns custom scalar data layer at given index (mutable).
	doubleArray& scalarDataAt(size_t idx);

	//! Returns custom vector data layer at given index (mutable).
	vectorArray& vectorDataAt(size_t idx);

private:
	size_t _numberOfParticles = 0;
	std::vector<Vector3> _positions;
	std::vector<Vector3> _velocities;
	std::vector<Vector3> _forces;
	std::vector<double> _densities;
	std::vector<double> _pressures;

	std::vector<double> _waterContent;
	std::vector<double> _sedimentCarried;
	
	//! Target density of this particle system in kg/m^3.
	double _targetDensity = 1000.0;

	double _radius = 1e-3;
	double _mass = 10;

	PointNeighbourSearcherPtr _neighbourSearcher;
	std::vector<std::vector<size_t>> _neighbourLists;

	std::vector<vectorArray> _vectorDataList;
	std::vector<doubleArray> _scalarDataList;
};

typedef std::shared_ptr<ParticleSystemData> ParticleSystemDataPtr;

#endif