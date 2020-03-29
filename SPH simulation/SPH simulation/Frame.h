#pragma once
#ifndef INCLUDE_FRAME_H_
#define INCLUDE_FRAME_H_

struct Frame final
{
	int index = 0;
	double timeIntervalInSeconds = 1.0/60.0;

	Frame()	{}
	Frame(int newIndex, double newTimeIntervalInSeconds)
	{
		index = newIndex;
		timeIntervalInSeconds = newTimeIntervalInSeconds;
	}

	double timeInSeconds() const
	{
		return index * timeIntervalInSeconds;
	}
	void advance()
	{
		index++;
	}
	void advance(unsigned int delta)
	{
		index += delta;
	}

	//! Advances single frame (prefix).
	Frame& operator++()
	{
		advance();
		return *this;
	}
};
#endif