#pragma once
#ifndef INCLUDE_ANIMATION_H_
#define INCLUDE_ANIMATION_H_

#include "Frame.h"
class Animation
{
public:
	void Update(const Frame& frame)
	{
		//pre-processing
		onUpdate(frame);
		//post-processing
	}
protected:
	virtual void onUpdate(const Frame& frame) = 0;
};

#endif