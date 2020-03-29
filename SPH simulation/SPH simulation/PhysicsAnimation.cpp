#include "PhysicsAnimation.h"

#include <limits>

#include "Timer.h"

PhysicsAnimation::PhysicsAnimation()
{
	_currentFrame.index = -1;
}

double PhysicsAnimation::currentTimeInSeconds() const
{
	return _currentTime;
}

unsigned int PhysicsAnimation::numberOfSubTimeSteps(double timeIntervalInSeconds) const
{
	// Returns number of fixed sub-timesteps by default
	return _numberOfFixedSubTimeSteps;
}

void PhysicsAnimation::onInitialise()
{
}

void PhysicsAnimation::onUpdate(const Frame& frame)
{
	if (frame.index > _currentFrame.index)
	{
		if (_currentFrame.index < 0)
		{
			initialise();
		}
		unsigned int numberOfFrames = frame.index - _currentFrame.index;

		for (unsigned int i = 0; i < numberOfFrames; i++)
		{
			advanceTimeStep(frame.timeIntervalInSeconds);
		}
		_currentFrame = frame;
	}
}

void PhysicsAnimation::advanceTimeStep(double timeIntervalInSeconds)
{
	_currentTime = _currentFrame.timeInSeconds();

	if (_isUsingFixedSubTimeSteps) 
	{
		// Perform fixed time-stepping
		const double actualTimeInterval =
			timeIntervalInSeconds /
			static_cast<double>(_numberOfFixedSubTimeSteps);

		for (unsigned int i = 0; i < _numberOfFixedSubTimeSteps; ++i) 
		{

			Timer timer;
			onAdvanceTimeStep(actualTimeInterval);

			_currentTime += actualTimeInterval;
		}
	}
	else 
	{

		// Perform adaptive time-stepping
		double remainingTime = timeIntervalInSeconds;
		while (remainingTime > std::numeric_limits<double>::epsilon()) {
			unsigned int numSteps = numberOfSubTimeSteps(remainingTime);
			double actualTimeInterval =
				remainingTime / static_cast<double>(numSteps);

			Timer timer;
			onAdvanceTimeStep(actualTimeInterval);

			remainingTime -= actualTimeInterval;
			_currentTime += actualTimeInterval;
		}
	}
}

void PhysicsAnimation::initialise()
{
	onInitialise();
}
