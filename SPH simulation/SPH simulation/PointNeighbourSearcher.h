#pragma once
#ifndef INCLUDE_POINT_NEIGHBOR_SEARCHER_H_
#define INCLUDE_POINT_NEIGHBOR_SEARCHER_H_

#include <functional>
#include <vector>
#include <memory>

#include "Vector3.h"

class PointNeighbourSearcher
{
public:
	typedef std::function<void(size_t, const Vector3&)> ForEachNearbyPointFunc;
	PointNeighbourSearcher();
	virtual ~PointNeighbourSearcher();

	virtual void build(std::vector<Vector3>& points) = 0;

	virtual void forEachNearbyPoint(
		Vector3& origin,
		double radius,
		const ForEachNearbyPointFunc& callback) = 0;

	virtual bool hasNearbyPoint(
		const Vector3& origin, double radius) = 0;
};

typedef std::shared_ptr<PointNeighbourSearcher> PointNeighbourSearcherPtr;
#endif