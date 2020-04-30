#pragma once
#ifndef INCLUDE_PHYSICSANIMATION_H_
#define INCLUDE_PHYSICSANIMATION_H_

#include "Animation.h"

class PhysicsAnimation : public Animation
{
public:
	//! Default constructor.
	PhysicsAnimation();
protected:
	virtual void onAdvanceTimeStep(double timeIntervalInSeconds) = 0;

	//!
	//! \brief      Returns current time in seconds.
	//!
	//! This function returns the current time which is calculated by adding
	//! current frame + sub-timesteps it passed.
	//!
	double currentTimeInSeconds() const;

	virtual unsigned int numberOfSubTimeSteps(
		double timeIntervalInSeconds) const;

	virtual void onInitialise();
private:
	Frame _currentFrame;

	double _currentTime = 0.0;

	bool _isUsingFixedSubTimeSteps = false;

	unsigned int _numberOfFixedSubTimeSteps = 1;

	void onUpdate(const Frame& frame) final;

	void advanceTimeStep(double timeIntervalInSeconds);

	void initialise();
};

#endif