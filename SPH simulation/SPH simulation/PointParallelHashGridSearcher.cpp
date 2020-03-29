#include "PointParallelHashGridSearcher.h"
#include <algorithm>
#include <cmath>

PointParallelHashGridSearcher::PointParallelHashGridSearcher(const Vector3 & resolution, double gridSpacing)
	:	PointParallelHashGridSearcher(
		resolution.x, resolution.y, resolution.z, gridSpacing) {}

PointParallelHashGridSearcher::PointParallelHashGridSearcher(size_t resolutionX, size_t resolutionY, size_t resolutionZ, double gridSpacing)
	:
	_gridSpacing(gridSpacing) 
{
	_resolution.x = std::max(static_cast<size_t>(resolutionX), size_t(1));
	_resolution.y = std::max(static_cast<size_t>(resolutionY), size_t(1));
	_resolution.z = std::max(static_cast<size_t>(resolutionZ), size_t(1));

	_startIndexTable.resize(
		_resolution.x * _resolution.y * _resolution.z, std::numeric_limits<size_t>::max());
	_endIndexTable.resize(
		_resolution.x * _resolution.y * _resolution.z, std::numeric_limits<size_t>::max());
}

void PointParallelHashGridSearcher::build(std::vector<Vector3>& points)
{
	_points.clear();
	_keys.clear();
	_startIndexTable.clear();
	_endIndexTable.clear();
	_sortedIndices.clear();

	// Allocate memory chuncks
	size_t numberOfPoints = points.size();
	std::vector<size_t> tempKeys(numberOfPoints);
	_startIndexTable.resize(_resolution.x * _resolution.y * _resolution.z);
	_endIndexTable.resize(_resolution.x * _resolution.y * _resolution.z);
	for (size_t i = 0; i < _startIndexTable.size(); i++)
	{
		_startIndexTable[i] = std::numeric_limits<size_t>::max();
	}
	for (size_t i = 0; i < _endIndexTable.size(); i++)
	{
		_endIndexTable[i] = std::numeric_limits<size_t>::max();
	}
	_keys.resize(numberOfPoints);
	_sortedIndices.resize(numberOfPoints);
	_points.resize(numberOfPoints);

	if (numberOfPoints == 0)
	{
		return;
	}

	// Initialize indices array and generate hash key for each point
	for (size_t i = 0; i < numberOfPoints; i++)
	{
		_sortedIndices[i] = i;
		_points[i] = points[i];
		tempKeys[i] = getHashKeyFromPosition(points[i]);
	}

	size_t size = static_cast<size_t>(_sortedIndices.end() - _sortedIndices.begin());
	// Sort indices based on hash key
	std::sort(
		_sortedIndices.begin(),
		_sortedIndices.begin() + size,
		[&tempKeys](size_t indexA, size_t indexB) 
	{
		return tempKeys[indexA] < tempKeys[indexB];
	});

	// Re-order point and key arrays
	for (size_t i = 0; i < numberOfPoints; i++)
	{
		_points[i] = points[_sortedIndices[i]];
		_keys[i] = tempKeys[_sortedIndices[i]];
	}

	// Now _points and _keys are sorted by points' hash key values.
	// Let's fill in start/end index table with _keys.

	// Assume that _keys array looks like:
	// [5|8|8|10|10|10]
	// Then _startIndexTable and _endIndexTable should be like:
	// [.....|0|...|1|..|3|..]
	// [.....|1|...|3|..|6|..]
	//       ^5    ^8   ^10
	// So that _endIndexTable[i] - _startIndexTable[i] is the number points
	// in i-th table bucket.

	_startIndexTable[_keys[0]] = 0;
	_endIndexTable[_keys[numberOfPoints - 1]] = numberOfPoints;

	for (size_t i = 1; i < numberOfPoints; i++)
	{
		if (_keys[i] > _keys[i - 1])
		{
			_startIndexTable[_keys[i]] = i;
			_endIndexTable[_keys[i - 1]] = i;
		}
	}

	size_t sumNumberOfPointsPerBucket = 0;
	size_t maxNumberOfPointsPerBucket = 0;
	size_t numberOfNonEmptyBucket = 0;
	for (size_t i = 0; i < _startIndexTable.size(); ++i) {
		if (_startIndexTable[i] != std::numeric_limits<size_t>::max()) {
			size_t numberOfPointsInBucket
				= _endIndexTable[i] - _startIndexTable[i];
			sumNumberOfPointsPerBucket += numberOfPointsInBucket;
			maxNumberOfPointsPerBucket =
				std::max(maxNumberOfPointsPerBucket, numberOfPointsInBucket);
			++numberOfNonEmptyBucket;
		}
	}
}

void PointParallelHashGridSearcher::forEachNearbyPoint(Vector3 & origin, double radius, const ForEachNearbyPointFunc & callback)
{
	size_t nearbyKeys[8];
	getNearbyKeys(origin, nearbyKeys);

	const double queryRadiusSquared = radius * radius;

	for (int i = 0; i < 8; i++) {
		size_t nearbyKey = nearbyKeys[i];
		size_t start = _startIndexTable[nearbyKey];
		size_t end = _endIndexTable[nearbyKey];

		// Empty bucket -- continue to next bucket
		if (start == std::numeric_limits<size_t>::max()) {
			continue;
		}

		for (size_t j = start; j < end; ++j) {
			Vector3 direction = _points[j].vectorSubtract(origin);
			double distanceSquared = direction.lengthSquared();
			if (distanceSquared <= queryRadiusSquared) {
				double distance = 0.0;
				if (distanceSquared > 0) {
					distance = std::sqrt(distanceSquared);
					direction = direction.scalarDivide(distance);
				}

				callback(_sortedIndices[j], _points[j]);
			}
		}
	}
}

bool PointParallelHashGridSearcher::hasNearbyPoint(const Vector3 & origin, double radius)
{
	size_t nearbyKeys[8];
	getNearbyKeys(origin, nearbyKeys);

	const double queryRadiusSquared = radius * radius;

	for (int i = 0; i < 8; i++) {
		size_t nearbyKey = nearbyKeys[i];
		size_t start = _startIndexTable[nearbyKey];
		size_t end = _endIndexTable[nearbyKey];

		// Empty bucket -- continue to next bucket
		if (start == std::numeric_limits<size_t>::max()) {
			continue;
		}

		for (size_t j = start; j < end; ++j) {
			Vector3 direction = _points[j].vectorSubtract(origin);
			double distanceSquared = direction.lengthSquared();
			if (distanceSquared <= queryRadiusSquared) {
				return true;
			}
		}
	}

	return false;
}

const std::vector<size_t>& PointParallelHashGridSearcher::keys() const
{
	return _keys;
}

const std::vector<size_t>& PointParallelHashGridSearcher::startIndexTable() const
{
	return _startIndexTable;
}

const std::vector<size_t>& PointParallelHashGridSearcher::endIndexTable() const
{
	return _endIndexTable;
}

const std::vector<size_t>& PointParallelHashGridSearcher::sortedIndices() const
{
	return _sortedIndices;
}

size_t PointParallelHashGridSearcher::getHashKeyFromBucketIndex(const Vector3 & bucketIndex) const
{
	Vector3 wrappedIndex = bucketIndex;
	wrappedIndex.x = fmod(bucketIndex.x, _resolution.x);
	wrappedIndex.y = fmod(bucketIndex.y, _resolution.y);
	wrappedIndex.z = fmod(bucketIndex.z, _resolution.z);
	if (wrappedIndex.x < 0) {
		wrappedIndex.x += _resolution.x;
	}
	if (wrappedIndex.y < 0) {
		wrappedIndex.y += _resolution.y;
	}
	if (wrappedIndex.z < 0) {
		wrappedIndex.z += _resolution.z;
	}
	return static_cast<size_t>(
		(wrappedIndex.z * _resolution.y + wrappedIndex.y) * _resolution.x
		+ wrappedIndex.x);
}

Vector3 PointParallelHashGridSearcher::getBucketIndex(const Vector3 & position) const
{
	Vector3 bucketIndex;
	bucketIndex.x = static_cast<size_t>(
		std::floor(position.x / _gridSpacing));
	bucketIndex.y = static_cast<size_t>(
		std::floor(position.y / _gridSpacing));
	bucketIndex.z = static_cast<size_t>(
		std::floor(position.z / _gridSpacing));
	return bucketIndex;
}

size_t PointParallelHashGridSearcher::getHashKeyFromPosition(const Vector3 & position) const
{
	Vector3 bucketIndex = getBucketIndex(position);

	return getHashKeyFromBucketIndex(bucketIndex);
}

void PointParallelHashGridSearcher::getNearbyKeys(const Vector3 & position, size_t * bucketIndices) const
{
	Vector3 originIndex = getBucketIndex(position), nearbyBucketIndices[8];

	for (int i = 0; i < 8; i++) {
		nearbyBucketIndices[i] = originIndex;
	}

	if ((originIndex.x + 0.5f) * _gridSpacing <= position.x) {
		nearbyBucketIndices[4].x += 1;
		nearbyBucketIndices[5].x += 1;
		nearbyBucketIndices[6].x += 1;
		nearbyBucketIndices[7].x += 1;
	}
	else {
		nearbyBucketIndices[4].x -= 1;
		nearbyBucketIndices[5].x -= 1;
		nearbyBucketIndices[6].x -= 1;
		nearbyBucketIndices[7].x -= 1;
	}

	if ((originIndex.y + 0.5f) * _gridSpacing <= position.y) {
		nearbyBucketIndices[2].y += 1;
		nearbyBucketIndices[3].y += 1;
		nearbyBucketIndices[6].y += 1;
		nearbyBucketIndices[7].y += 1;
	}
	else {
		nearbyBucketIndices[2].y -= 1;
		nearbyBucketIndices[3].y -= 1;
		nearbyBucketIndices[6].y -= 1;
		nearbyBucketIndices[7].y -= 1;
	}

	if ((originIndex.z + 0.5f) * _gridSpacing <= position.z) {
		nearbyBucketIndices[1].z += 1;
		nearbyBucketIndices[3].z += 1;
		nearbyBucketIndices[5].z += 1;
		nearbyBucketIndices[7].z += 1;
	}
	else {
		nearbyBucketIndices[1].z -= 1;
		nearbyBucketIndices[3].z -= 1;
		nearbyBucketIndices[5].z -= 1;
		nearbyBucketIndices[7].z -= 1;
	}

	for (int i = 0; i < 8; i++) {
		bucketIndices[i] = getHashKeyFromBucketIndex(nearbyBucketIndices[i]);
	}
}
