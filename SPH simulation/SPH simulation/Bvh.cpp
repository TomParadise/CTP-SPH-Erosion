#include "Bvh.h"
#include <numeric>
#include <algorithm>

using ContainerType = std::vector<size_t>;
using Iterator = ContainerType::iterator;
using ConstIterator = ContainerType::const_iterator;

Bvh::Bvh(){}

void Bvh::build(const std::vector<size_t>& items, const std::vector<BoundingBox>& itemsBounds)
{
	_items = items;
	_itemBounds = itemsBounds;

	if (_items.empty()) {
		return;
	}

	_nodes.clear();
	_bound = BoundingBox();

	for (size_t i = 0; i < _items.size(); ++i) 
	{
		_bound.merge(_itemBounds[i]);
	}

	std::vector<size_t> itemIndices(_items.size());
	std::iota(std::begin(itemIndices), std::end(itemIndices), 0);

	build(0, itemIndices.data(), _items.size(), 0);
}

NearestNeighborQueryResult Bvh::nearest(const Vector3 & pt, const NearestNeighborDistanceFunc & distanceFunc) const
{
		NearestNeighborQueryResult best;
		best.distance = std::numeric_limits<double>::max();
		best.item = nullptr;

		// Prepare to traverse BVH
		static const int kMaxTreeDepth = 8 * sizeof(size_t);
		const Node* todo[kMaxTreeDepth];
		size_t todoPos = 0;

		// Traverse BVH nodes
		const Node* node = _nodes.data();
		while (node != nullptr)
		{
			if (node->isLeaf()) 
			{
				double dist = distanceFunc(_items[node->item], pt);
				if (dist < best.distance) 
				{
					best.distance = dist;
					best.item = &_items[node->item];
				}

				// Grab next node to process from todo stack
				if (todoPos > 0) 
				{
					// Dequeue
					--todoPos;
					node = todo[todoPos];
				}
				else
				{
					break;
				}
			}
			else 
			{
				const double bestDistSqr = best.distance * best.distance;

				const Node* left = node + 1;
				const Node* right = &_nodes[node->child];

				// If pt is inside the box, then the closestLeft and Right will be
				// identical to pt. This will make distMinLeftSqr and
				// distMinRightSqr zero, meaning that such a box will have higher
				// priority.
				Vector3 closestLeft = left->bound.clamp(pt);
				Vector3 closestRight = right->bound.clamp(pt);

				double distMinLeftSqr = closestLeft.distanceSquaredTo(pt);
				double distMinRightSqr = closestRight.distanceSquaredTo(pt);

				bool shouldVisitLeft = distMinLeftSqr < bestDistSqr;
				bool shouldVisitRight = distMinRightSqr < bestDistSqr;

				const Node* firstChild;
				const Node* secondChild;
				if (shouldVisitLeft && shouldVisitRight) 
				{
					if (distMinLeftSqr < distMinRightSqr) 
					{
						firstChild = left;
						secondChild = right;
					}
					else 
					{
						firstChild = right;
						secondChild = left;
					}

					// Enqueue secondChild in todo stack
					todo[todoPos] = secondChild;
					++todoPos;
					node = firstChild;
				}
				else if (shouldVisitLeft)
				{
					node = left;
				}
				else if (shouldVisitRight) 
				{
					node = right;
				}
				else 
				{
					if (todoPos > 0) 
					{
						// Dequeue
						--todoPos;
						node = todo[todoPos];
					}
					else
					{
						break;
					}
				}
			}
		}

		return best;
}

const BoundingBox & Bvh::boundingBox() const
{
	return _bound;
}

size_t Bvh::numberOfItems() const
{
	return _items.size();
}

size_t Bvh::numberOfNodes() const
{
	return _nodes.size();
}

std::pair<size_t, size_t> Bvh::children(size_t i) const
{
	if (isLeaf(i)) 
	{
		return std::make_pair(std::numeric_limits<size_t>::max(), std::numeric_limits<size_t>::max());
	}
	else 
	{
		return std::make_pair(i + 1, _nodes[i].child);
	}
}

bool Bvh::isLeaf(size_t i) const
{
	return _nodes[i].isLeaf();
}

const BoundingBox & Bvh::nodeBound(size_t i) const
{
	return _nodes[i].bound;
}

Iterator Bvh::itemOfNode(size_t i)
{
	if (isLeaf(i)) 
	{
		return _nodes[i].item + _items.begin();
	}
	else 
	{
		return _items.end();
	}
}

size_t Bvh::build(size_t nodeIndex, size_t * itemIndices, size_t nItems, size_t currentDepth)
{ 
	// add a node
	_nodes.push_back(Node());

	// initialize leaf node if termination criteria met
	if (nItems == 1)
	{
		_nodes[nodeIndex].initLeaf(itemIndices[0], _itemBounds[itemIndices[0]]);
		return currentDepth + 1;
	}

	// find the mid-point of the bounding box to use as a qsplit pivot
	BoundingBox nodeBound;
	for (size_t i = 0; i < nItems; ++i) 
	{
		nodeBound.merge(_itemBounds[itemIndices[i]]);
	}

	Vector3 d = nodeBound.upperCorner - nodeBound.lowerCorner;

	// choose which axis to split along
	uint8_t axis;
	double cornerVal;
	if (d.x > d.y && d.x > d.z) 
	{
		axis = 0;
		cornerVal = nodeBound.upperCorner.x + nodeBound.lowerCorner.x;
	}
	else
	{
		axis = (d.y > d.z) ? 1 : 2;
		if (axis == 1)
		{
			cornerVal = nodeBound.upperCorner.y + nodeBound.lowerCorner.y;
		}
		else
		{
			cornerVal = nodeBound.upperCorner.z + nodeBound.lowerCorner.z;
		}
	}

	double pivot = 0.5 * cornerVal;

	// classify primitives with respect to split
	size_t midPoint = qsplit(itemIndices, nItems, pivot, axis);

	// recursively initialize children _nodes
	size_t d0 = build(nodeIndex + 1, itemIndices, midPoint, currentDepth + 1);
	_nodes[nodeIndex].initInternal(axis, _nodes.size(), nodeBound);
	size_t d1 = build(_nodes[nodeIndex].child, itemIndices + midPoint,
		nItems - midPoint, currentDepth + 1);

	return std::max(d0, d1);
}

size_t Bvh::qsplit(size_t * itemIndices, size_t numItems, double pivot, uint8_t axis)
{
	double centroid;
	size_t ret = 0;
	for (size_t i = 0; i < numItems; ++i) 
	{
		BoundingBox b = _itemBounds[itemIndices[i]];

		double cornerVal;
		if (axis == 0)
		{
			cornerVal = b.lowerCorner.x + b.lowerCorner.x;
		}
		else if (axis == 1)
		{
			cornerVal = b.lowerCorner.y + b.lowerCorner.y;
		}
		else
		{
			cornerVal = b.lowerCorner.z + b.lowerCorner.z;
		}

		centroid = 0.5f * cornerVal;
		if (centroid < pivot) 
		{
			std::swap(itemIndices[i], itemIndices[ret]);
			ret++;
		}
	}
	if (ret == 0 || ret == numItems) 
	{
		ret = numItems >> 1;
	}
	return ret;
}

Bvh::Node::Node() : flags(0) 
{
	child = std::numeric_limits<size_t>::max();
}

void Bvh::Node::initLeaf(size_t it, const BoundingBox & b)
{
	flags = 3;
	item = it;
	bound = b;
}

void Bvh::Node::initInternal(uint8_t axis, size_t c, const BoundingBox & b)
{
	flags = axis;
	child = c;
	bound = b;
}

bool Bvh::Node::isLeaf() const
{
	return flags == 3;
}
