#pragma once
#ifndef INCLUDE_BCC_LATTICE_POINT_GENERATOR_H_
#define INCLUDE_BCC_LATTICE_POINT_GENERATOR_H_

#include "PointGenerator.h"

class BccLatticePointGenerator : public PointGenerator
{
public:
	void forEachPoint(
		const BoundingBox& boundingBox,
		double spacing,
		const std::function<bool(const Vector3&)>& callback) const override;
};

//! Shared pointer type for the BccLatticePointGenerator.
typedef std::shared_ptr<BccLatticePointGenerator> BccLatticePointGeneratorPtr;

#endif