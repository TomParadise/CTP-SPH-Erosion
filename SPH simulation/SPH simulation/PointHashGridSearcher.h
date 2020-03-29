#pragma once
#ifndef INCLUDE_POINT_HASH_GRID_SEARCHER_H_
#define INCLUDE_POINT_HASH_GRID_SEARCHER_H_

#include <vector>

#include "PointNeighbourSearcher.h"

class PointHashGridSearcher final : public PointNeighbourSearcher
{
public:
	PointHashGridSearcher(const Vector3 resolution, double gridSpacing);
	PointHashGridSearcher(
		size_t resolutionX,
		size_t resolutionY,
		size_t resolutionZ,
		double gridSpacing);

	void build(std::vector<Vector3>& points) override;

	void forEachNearbyPoint(
		Vector3& origin,
		double radius,
		const ForEachNearbyPointFunc& callback) override;

	Vector3 getBucketIndex(const Vector3& position) const;


	size_t getHashKeyFromBucketIndex(const Vector3& bucketIndex) const;


	bool hasNearbyPoint(
		const Vector3& origin, double radius) override;

	void add(const Vector3& point);

private:
	double _gridspacing = 1.0;
	Vector3 _resolution = Vector3(1, 1, 1);
	std::vector<Vector3> _points;
	std::vector< std::vector<size_t>> _buckets;

	size_t getHashKeyFromPosition(const Vector3& position) const;
	void getNearbyKeys(const Vector3& positon, size_t* nearbyKeys) const;
};

typedef std::shared_ptr<PointHashGridSearcher> PointHashGridSearcherPtr;
#endif