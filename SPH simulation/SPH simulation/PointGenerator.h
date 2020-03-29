#pragma once
#ifndef INCLUDE_POINT_GENERATOR_H_
#define INCLUDE_POINT_GENERATOR_H_

#include <memory>
#include <vector>
#include <functional>

#include "Vector3.h"
#include "BoundingBox.h"
class PointGenerator
{
public:
	PointGenerator();
	virtual ~PointGenerator();

	//! Generates points to output array \p points inside given \p boundingBox
	//! with target point \p spacing.
	void generate(
		const BoundingBox& boundingBox,
		double spacing,
		std::vector<Vector3>* points) const;

	//!
	//! \brief Iterates every point within the bounding box with specified
	//! point pattern and invokes the callback function.
	//!
	//! This function iterates every point within the bounding box and invokes
	//! the callback function. The position of the point is specified by the
	//! actual implementation. The suggested spacing between the points are
	//! given by \p spacing. The input parameter of the callback function is
	//! the position of the point and the return value tells whether the
	//! iteration should stop or not.
	//!
	virtual void forEachPoint(
		const BoundingBox& boundingBox,
		double spacing,
		const std::function<bool(const Vector3&)>& callback) const = 0;
};

//! Shared pointer for the PointGenerator3 type.
typedef std::shared_ptr<PointGenerator> PointGeneratorPtr;

#endif