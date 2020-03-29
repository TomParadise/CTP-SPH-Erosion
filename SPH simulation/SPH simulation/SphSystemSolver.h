#pragma once
#ifndef INCLUDE_SPH_SOLVER_H_
#define INCLUDE_SPH_SOLVER_H_

#include "ParticleSystemSolver.h"
#include "SphSystemData.h"

class SphSystemSolver : public ParticleSystemSolver
{
public:
	SphSystemSolver();

	//! Constructs a solver with target density, spacing, and relative kernel
	//! radius.
	SphSystemSolver(
		double targetDensity,
		double targetSpacing,
		double relativeKernelRadius);

	virtual ~SphSystemSolver();

	//! Returns the negative pressure scale.
	double negativePressureScale() const;

	//!
	//! \brief Sets the negative pressure scale.
	//!
	//! This function sets the negative pressure scale. By setting the number
	//! between 0 and 1, the solver will scale the effect of negative pressure
	//! which can prevent the clumping of the particles near the surface. Input
	//! value outside 0 and 1 will be clamped within the range. Default is 0.
	//!
	void setNegativePressureScale(double newNegativePressureScale);

	void setPseudoViscosityCoefficient(double newPseudoViscosityCoefficient);

	void setTimeStepLimitScale(double newScale);

	SphSystemDataPtr sphSystemData() const;

protected:
	unsigned int numberOfSubTimeSteps(
		double timeIntervalInSeconds) const override;

	void accumulateForces(double timeStepInSeconds) override;

	void onBeginAdvanceTimeStep(double timeStepInSeconds) override;

	void onEndAdvanceTimeStep(double timeStepInSeconds) override;

	virtual void accumulateNonPressureForces(double timeStepInSeconds);

	virtual void accumulatePressureForce(double timeStepInSeconds);

	void accumulatePressureForce(
		const std::vector<Vector3>& positions,
		const std::vector<double>& densities,
		const std::vector<double>& pressures,
		std::vector<Vector3>& pressureForces);

	void computePressure();

	void accumulateViscosityForce();

	void computePseudoViscosity(double timeStepInSeconds);

	double computePressureFromEos(double density, double targetDensity, double eosScale, double eosExponent, double negativePressureScale);

private:

	//! Viscosity coefficient.
	double _viscosityCoefficient = 0.01;

	//! Pseudo-viscosity coefficient velocity filtering.
	//! This is a minimum "safety-net" for SPH solver which is quite
	//! sensitive to the parameters.
	double _pseudoViscosityCoefficient = 10.0;

	double _speedOfSound = 100.0;
	//! Exponent component of equation-of-state (or Tait's equation).
	double _eosExponent = 7.0;

	//! Negative pressure scaling factor.
	//! Zero means clamping. One means do nothing.
	double _negativePressureScale = 0.0;

	//! Scales the max allowed time-step.
	double _timeStepLimitScale = 5.0;
};

typedef std::shared_ptr<SphSystemSolver> SphSystemSolverPtr;

#endif