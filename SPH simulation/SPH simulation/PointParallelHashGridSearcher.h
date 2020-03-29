#pragma once
#ifndef INCLUDE_POINT_PARALLEL_HASH_GRID_SEARCHER_H_
#define INCLUDE_POINT_PARALLEL_HASH_GRID_SEARCHER_H_

#include "PointNeighbourSearcher.h"
#include "Vector3.h"
#include <vector>

//!
//! \brief Parallel version of hash grid-based 3-D point searcher.
//!
//! This class implements parallel version of 3-D point searcher by using hash
//! grid for its internal acceleration data structure. Each point is recorded to
//! its corresponding bucket where the hashing function is 3-D grid mapping.
//!
class PointParallelHashGridSearcher final : public PointNeighbourSearcher 
{
public:
	PointParallelHashGridSearcher(
		const Vector3& resolution, double gridSpacing);

	//!
	//! \brief      Constructs hash grid with given resolution and grid spacing.
	//!
	//! This constructor takes hash grid resolution and its grid spacing as
	//! its input parameters. The grid spacing must be 2x or greater than
	//! search radius.
	//!
	//! \param[in]  resolutionX The resolution x.
	//! \param[in]  resolutionY The resolution y.
	//! \param[in]  resolutionZ The resolution z.
	//! \param[in]  gridSpacing The grid spacing.
	//!
	PointParallelHashGridSearcher(
		size_t resolutionX,
		size_t resolutionY,
		size_t resolutionZ,
		double gridSpacing);

	void build(std::vector<Vector3>& points) override;

	//!
	//! Invokes the callback function for each nearby point around the origin
	//! within given radius.
	//!
	//! \param[in]  origin   The origin position.
	//! \param[in]  radius   The search radius.
	//! \param[in]  callback The callback function.
	//!
	void forEachNearbyPoint(
		Vector3& origin,
		double radius,
		const ForEachNearbyPointFunc& callback) override;

	//!
	//! Returns true if there are any nearby points for given origin within
	//! radius.
	//!
	//! \param[in]  origin The origin.
	//! \param[in]  radius The radius.
	//!
	//! \return     True if has nearby point, false otherwise.
	//!
	bool hasNearbyPoint(
		const Vector3& origin, double radius) override;

	//!
	//! \brief      Returns the hash key list.
	//!
	//! The hash key list maps sorted point index i to its hash key value.
	//! The sorting order is based on the key value itself.
	//!
	//! \return     The hash key list.
	//!
	const std::vector<size_t>& keys() const;

	//!
	//! \brief      Returns the start index table.
	//!
	//! The start index table maps the hash grid bucket index to starting index
	//! of the sorted point list. Assume the hash key list looks like:
	//!
	//! \code
	//! [5|8|8|10|10|10]
	//! \endcode
	//!
	//! Then startIndexTable and endIndexTable should be like:
	//!
	//! \code
	//! [.....|0|...|1|..|3|..]
	//! [.....|1|...|3|..|6|..]
	//!       ^5    ^8   ^10
	//! \endcode
	//!
	//! So that endIndexTable[i] - startIndexTable[i] is the number points
	//! in i-th table bucket.
	//!
	//! \return     The start index table.
	//!
	const std::vector<size_t>& startIndexTable() const;

	//!
	//! \brief      Returns the end index table.
	//!
	//! The end index table maps the hash grid bucket index to starting index
	//! of the sorted point list. Assume the hash key list looks like:
	//!
	//! \code
	//! [5|8|8|10|10|10]
	//! \endcode
	//!
	//! Then startIndexTable and endIndexTable should be like:
	//!
	//! \code
	//! [.....|0|...|1|..|3|..]
	//! [.....|1|...|3|..|6|..]
	//!       ^5    ^8   ^10
	//! \endcode
	//!
	//! So that endIndexTable[i] - startIndexTable[i] is the number points
	//! in i-th table bucket.
	//!
	//! \return     The end index table.
	//!
	const std::vector<size_t>& endIndexTable() const;

	//!
	//! \brief      Returns the sorted indices of the points.
	//!
	//! When the hash grid is built, it sorts the points in hash key order. But
	//! rather than sorting the original points, this class keeps the shuffled
	//! indices of the points. The list this function returns maps sorted index
	//! i to original index j.
	//!
	//! \return     The sorted indices of the points.
	//!
	const std::vector<size_t>& sortedIndices() const;

	//!
	//! Returns the hash value for given 3-D bucket index.
	//!
	//! \param[in]  bucketIndex The bucket index.
	//!
	//! \return     The hash key from bucket index.
	//!
	size_t getHashKeyFromBucketIndex(const Vector3& bucketIndex) const;

	//!
	//! Gets the bucket index from a point.
	//!
	//! \param[in]  position The position of the point.
	//!
	//! \return     The bucket index.
	//!
	Vector3 getBucketIndex(const Vector3& position) const;

private:
	double _gridSpacing = 1.0;
	Vector3 _resolution = Vector3(1, 1, 1);
	std::vector<Vector3> _points;
	std::vector<size_t> _keys;
	std::vector<size_t> _startIndexTable;
	std::vector<size_t> _endIndexTable;
	std::vector<size_t> _sortedIndices;

	size_t getHashKeyFromPosition(const Vector3& position) const;

	void getNearbyKeys(const Vector3& position, size_t* bucketIndices) const;
};

//! Shared pointer for the PointParallelHashGridSearcher3 type.
typedef std::shared_ptr<PointParallelHashGridSearcher>
	PointParallelHashGridSearcherPtr;

#endif