#pragma once
#ifndef INCLUDE_BVH_H_
#define INCLUDE_BVH_H_

#include <limits>
#include <functional>
#include <vector>

#include "Vector3.h"
#include "BoundingBox.h"

struct NearestNeighborQueryResult 
{
const size_t* item = nullptr;
double distance = std::numeric_limits<double>::max();
};
using NearestNeighborDistanceFunc = std::function<double(const size_t&, const Vector3&)>;

class Bvh
{
public:
	using ContainerType = std::vector<size_t>;
	using Iterator = ContainerType::iterator;
	using ConstIterator = ContainerType::const_iterator;

	Bvh();

	//! Builds bounding volume hierarchy.
	void build(const std::vector<size_t>& items,
		const std::vector<BoundingBox>& itemsBounds);

	//! Returns the nearest neighbor for given point and distance measure
	//! function.
	NearestNeighborQueryResult nearest(
		const Vector3& pt,
		const NearestNeighborDistanceFunc& distanceFunc) const;

	//! Returns bounding box of every items.
	const BoundingBox& boundingBox() const;

	//! Returns the number of items.
	size_t numberOfItems() const;
	//! Returns the number of nodes.
	size_t numberOfNodes() const;

	//! Returns the children indices of \p i-th node.
	std::pair<size_t, size_t> children(size_t i) const;

	//! Returns true if \p i-th node is a leaf node.
	bool isLeaf(size_t i) const;

	//! Returns bounding box of \p i-th node.
	const BoundingBox& nodeBound(size_t i) const;

	//! Returns item of \p i-th node.
	Iterator itemOfNode(size_t i); 

private:
	struct Node 
	{
		char flags;
		union 
		{
			size_t child;
			size_t item;
		};
		BoundingBox bound;
	
		Node();
		void initLeaf(size_t it, const BoundingBox& b);
		void initInternal(uint8_t axis, size_t c, const BoundingBox& b);
		bool isLeaf() const;
	};
	
	BoundingBox _bound;
	ContainerType _items;
	std::vector<BoundingBox> _itemBounds;
	std::vector<Node> _nodes;
	
	size_t build(size_t nodeIndex, size_t* itemIndices, size_t nItems,
		size_t currentDepth);
	
	size_t qsplit(size_t* itemIndices, size_t numItems, double pivot,
		uint8_t axis);
};

#endif