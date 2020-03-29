#pragma once
#ifndef INCLUDE_PCI_SPH_SOLVER_H_
#define INCLUDE_PCI_SPH_SOLVER_H_

#include "SphSystemSolver.h"
#include "BccLatticePointGenerator.h"

class PciSphSystemSolver : public SphSystemSolver
{
public:
	class Builder;

	PciSphSystemSolver();

	//! Constructs a solver with target density, spacing, and relative kernel
	//! radius.
	PciSphSystemSolver(double targetDensity,double targetSpacing,double relativeKernelRadius);

	virtual ~PciSphSystemSolver();

protected:
	void accumulatePressureForce(double timeIntervalInSeconds) override;

	void onBeginAdvanceTimeStep(double timeStepInSeconds) override;

private:
	double _maxDensityErrorRatio = 0.01;
	unsigned int _maxNumberOfIterations = 5;

	ParticleSystemData::vectorArray _tempPositions;
	ParticleSystemData::vectorArray _tempVelocities;
	ParticleSystemData::vectorArray _pressureForces;
	ParticleSystemData::doubleArray _densityErrors;

	double computeDelta(double timeStepInSeconds);
	double computeBeta(double timeStepInSeconds);
};
//! Shared pointer type for the PciSphSolver3.
typedef std::shared_ptr<PciSphSystemSolver> PciSphSystemSolverPtr;

//!
//! \brief Front-end to create SphSolver3 objects step by step.
//!
class PciSphSystemSolver::Builder 
{
public:
	//! Returns builder with target density.
	Builder withTargetDensity(double targetDensity) { _targetDensity = targetDensity; return *this; }

	//! Returns builder with target spacing.
	Builder withTargetSpacing(double targetSpacing) { _targetSpacing = targetSpacing; return *this; }

	//! Returns builder with relative kernel radius.
	Builder withRelativeKernelRadius(double relativeKernelRadius) { _relativeKernelRadius = relativeKernelRadius; return *this; }

	//! Builds SphSolver3.
	PciSphSystemSolver build() const;

	//! Builds shared pointer of SphSolver3 instance.
	PciSphSystemSolverPtr makeShared() const;
protected:
	double _targetDensity = 1000;
	double _targetSpacing = 0.1;
	double _relativeKernelRadius = 1.8;
};

#endif