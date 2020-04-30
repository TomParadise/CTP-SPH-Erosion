#include "PointHashGridSearcher.h"

#include <algorithm>
#include <cmath>

PointHashGridSearcher::PointHashGridSearcher(const Vector3 resolution, double gridSpacing)
	:
	PointHashGridSearcher(
		resolution.x,
		resolution.y,
		resolution.z,
		gridSpacing){}

PointHashGridSearcher::PointHashGridSearcher(size_t resolutionX, size_t resolutionY, size_t resolutionZ, double gridSpacing)
	:
	_gridspacing(gridSpacing) 
{
	_resolution.x = std::max(static_cast<size_t>(resolutionX), (size_t)1);
	_resolution.y = std::max(static_cast<size_t>(resolutionY), (size_t)1);
	_resolution.z = std::max(static_cast<size_t>(resolutionZ), (size_t)1);
}

void PointHashGridSearcher::build(std::vector<Vector3>& points)
{
	_buckets.clear();
	_points.clear();

	_buckets.resize(_resolution.x * _resolution.y * _resolution.z);
	_points.resize(points.size());

	if (points.size() == 0)
	{
		return;
	}


	for (size_t i = 0; i < points.size(); ++i)
	{
		_points[i] = points[i];
		size_t key = getHashKeyFromPosition(points[i]);
		_buckets[key].push_back(i);
	}
}

void PointHashGridSearcher::forEachNearbyPoint(Vector3 & origin, double radius, const ForEachNearbyPointFunc & callback)
{
	if (_buckets.empty())
	{
		return;
	}
	size_t nearbyKeys[8];
	getNearbyKeys(origin, nearbyKeys);

	const double queryRadiusSquared = radius * radius;

	for (int i = 0; i < 8; i++)
	{
		const auto& bucket = _buckets[nearbyKeys[i]];
		size_t numberOfPointsInBucket = bucket.size();

		for (size_t j = 0; j < numberOfPointsInBucket; ++j)
		{
			size_t pointIndex = bucket[j];
			double rSquared = (_points[pointIndex] - origin).lengthSquared();
			if (rSquared <= queryRadiusSquared)
			{
				callback(pointIndex, _points[pointIndex]);
			}
		}
	}
}

Vector3 PointHashGridSearcher::getBucketIndex(const Vector3 & position) const
{
	Vector3 bucketIndex;
	bucketIndex.x = static_cast<size_t>(std::floor(position.x / _gridspacing));
	bucketIndex.y = static_cast<size_t>(std::floor(position.y / _gridspacing));
	bucketIndex.z = static_cast<size_t>(std::floor(position.z / _gridspacing));
	return bucketIndex;
}

size_t PointHashGridSearcher::getHashKeyFromPosition(const Vector3 & position) const
{
	Vector3 bucketIndex = getBucketIndex(position);
	return getHashKeyFromBucketIndex(bucketIndex);
}

size_t PointHashGridSearcher::getHashKeyFromBucketIndex(const Vector3 & bucketIndex) const
{
	Vector3 wrappedIndex = bucketIndex;
	wrappedIndex.x = fmod(bucketIndex.x, _resolution.x);
	wrappedIndex.y = fmod(bucketIndex.y, _resolution.y);
	wrappedIndex.z = fmod(bucketIndex.z, _resolution.z);
	if (wrappedIndex.x < 0) { wrappedIndex.x += _resolution.x; }
	if (wrappedIndex.y < 0) { wrappedIndex.y += _resolution.y; }
	if (wrappedIndex.z < 0) { wrappedIndex.z += _resolution.z; }
	return static_cast<size_t>(
		(wrappedIndex.z * _resolution.y + wrappedIndex.y)*_resolution.x + wrappedIndex.x);
}

void PointHashGridSearcher::getNearbyKeys(const Vector3 & positon, size_t * nearbyKeys) const
{
	Vector3 originIndex = getBucketIndex(positon), nearbyBucketIndices[8];
	for (int i = 0; i < 8; i++)
	{
		nearbyBucketIndices[i] = originIndex;
	}

	if ((originIndex.x + 0.5f)* _gridspacing <= positon.x)
	{
		nearbyBucketIndices[4].x += 1; nearbyBucketIndices[5].x += 1;
		nearbyBucketIndices[6].x += 1; nearbyBucketIndices[7].x += 1;
	}
	else
	{
		nearbyBucketIndices[4].x -= 1; nearbyBucketIndices[5].x -= 1;
		nearbyBucketIndices[6].x -= 1; nearbyBucketIndices[7].x -= 1;
	}
	if ((originIndex.y + 0.5f) * _gridspacing <= positon.y)
	{
		nearbyBucketIndices[2].y += 1; nearbyBucketIndices[3].y += 1;
		nearbyBucketIndices[6].y += 1; nearbyBucketIndices[7].y += 1;
	}
	else
	{
		nearbyBucketIndices[2].y -= 1; nearbyBucketIndices[3].y -= 1;
		nearbyBucketIndices[6].y -= 1; nearbyBucketIndices[7].y -= 1;
	}
	if((originIndex.z + 0.5f) * _gridspacing <= positon.z)
	{
		nearbyBucketIndices[1].z += 1; nearbyBucketIndices[3].z += 1;
		nearbyBucketIndices[5].z += 1; nearbyBucketIndices[7].z += 1;
	}
	else
	{
		nearbyBucketIndices[1].z -= 1; nearbyBucketIndices[3].z -= 1;
		nearbyBucketIndices[5].z -= 1; nearbyBucketIndices[7].z -= 1;
	}

	for (int i = 0; i < 8; i++)
	{
		nearbyKeys[i] = getHashKeyFromBucketIndex(nearbyBucketIndices[i]);
	}
}

bool PointHashGridSearcher::hasNearbyPoint(const Vector3 & origin, double radius)
{
	if (_buckets.empty()) 
	{
		return false;
	}

	size_t nearbyKeys[8];
	getNearbyKeys(origin, nearbyKeys);

	const double queryRadiusSquared = radius * radius;

	for (int i = 0; i < 8; i++)
	{
		const auto& bucket = _buckets[nearbyKeys[i]];
		size_t numberOfPointsInBucket = bucket.size();

		for (size_t j = 0; j < numberOfPointsInBucket; ++j) 
		{
			size_t pointIndex = bucket[j];
			double rSquared = (_points[pointIndex] - origin).lengthSquared();
			if (rSquared <= queryRadiusSquared) 
			{
				return true;
			}
		}
	}

	return false;
}

void PointHashGridSearcher::add(const Vector3 & point)
{
	if (_buckets.empty()) 
	{
		std::vector<Vector3> arr = { point };
		build(arr);
	}
	else
	{
		size_t i = _points.size();
		_points.push_back(point);
		size_t key = getHashKeyFromPosition(point);
		_buckets[key].push_back(i);
	}
}
