#pragma once
#ifndef INCLUDE_HEIGHTFIELD_H_
#define INCLUDE_HEIGHTFIELD_H_

#include <vector>

#include "Quaternion.h"
#include "Surface.h"

#include <iostream>


class Heightfield : public Surface
{
public:
	class Builder;
	Heightfield(std::vector<Vector3> points, bool isNormalFlipped, size_t resolutionX, size_t resolutionZ, BoundingBox maxRegion);

	static Builder builder();

	void depositToNode(Vector3 pos, double amountToDeposit) override;
	double erodeNode(Vector3 pos, double amountToErode) override;

	std::vector<Vector3> getVertices() override;

protected:
	Vector3 closestPointLocal(Vector3 otherPoint) const override;

	double closestDistanceLocal(Vector3 otherPoint) override;

	BoundingBox boundingBoxLocal() const override;

	Vector3 closestNormalLocal(const Vector3& otherPoint) const override;

	bool isInsideLocal(Vector3 otherPoint) override;

private:

private:
	std::vector<Vector3> _points;
	bool _isNormalFlipped = false;
	size_t _resolution_x;
	size_t _resolution_z;
	BoundingBox _maxRegion;

	std::vector<std::vector<int>> erosionBrushIndices;
	std::vector<std::vector<double>> erosionBrushWeights;
	std::vector<int> secondArrayLength;
};

//! Shared pointer for the TriangleMesh3 type.
typedef std::shared_ptr<Heightfield> HeightfieldPtr;

class Heightfield::Builder
{
public:
	//! Returns builder with flipped normal flag.
	Builder& withIsNormalFlipped(bool isNormalFlipped);

	//! Returns builder with points.
	Builder& withPoints(const std::vector<Vector3>& points);

	//! Returns builder with points.
	Builder& withResolution(size_t resolutionX, size_t resolutionZ);

	//! Builds TriangleMesh3.
	Heightfield build() const;

	//! Builds shared pointer of TriangleMesh3 instance.
	HeightfieldPtr makeShared() const;

	Builder& withBox(BoundingBox maxRegion);

private:
	bool _isNormalFlipped = false;
	std::vector<Vector3> _points;
	size_t _resolution_x;
	size_t _resolution_z;
	BoundingBox _maxRegion;
};

#endif