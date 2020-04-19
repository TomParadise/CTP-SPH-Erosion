#include "TriangleMesh.h"
#include <functional>


constexpr double kDefaultFastWindingNumberAccuracy = 2.0;
constexpr double kPiD = 3.14159265358979323846264338327950288;

struct WindingNumberGatherData 
{
	double areaSums = 0;
	Vector3 areaWeightedNormalSums;
	Vector3 areaWeightedPositionSums;

	WindingNumberGatherData operator+(const WindingNumberGatherData& other)
	{
		WindingNumberGatherData sum;
		sum.areaSums = areaSums + other.areaSums;
		sum.areaWeightedNormalSums = areaWeightedNormalSums + other.areaWeightedNormalSums;
		sum.areaWeightedPositionSums = areaWeightedPositionSums + other.areaWeightedPositionSums;

		return sum;
	}
};
WindingNumberGatherData postOrderTraversal(
	const Bvh& bvh, 
	size_t nodeIndex, 
	const std::function<void(size_t, const WindingNumberGatherData&)>& visitorFunc,
	const std::function<WindingNumberGatherData(size_t)>& leafFunc,
	const WindingNumberGatherData& initGatherData) 
{
	WindingNumberGatherData data = initGatherData;

	if (bvh.isLeaf(nodeIndex)) 
	{
		data = leafFunc(nodeIndex);
	}
	else
	{
		const auto children = bvh.children(nodeIndex);
		data = data + postOrderTraversal(bvh, children.first, visitorFunc,
			leafFunc, initGatherData);
		data = data + postOrderTraversal(bvh, children.second, visitorFunc,
			leafFunc, initGatherData);
	}
	visitorFunc(nodeIndex, data);

	return data;
}

TriangleMesh::TriangleMesh(const Transform & transform, bool isNormalFlipped)
	: Surface(transform, isNormalFlipped) {}

TriangleMesh::TriangleMesh(const PointArray & points, 
						   const NormalArray & normals, 
						   const IndexArray & pointIndices, 
						   const IndexArray & normalIndices,
						   const Transform & transform_,
						   bool isNormalFlipped)
	: Surface(transform_, isNormalFlipped),
	_points(points),
	_normals(normals),
	_pointIndices(pointIndices),
	_normalIndices(normalIndices){}

bool TriangleMesh::hasNormals() const
{
	return _normals.size() > 0;
}

void TriangleMesh::updateQueryEngine()
{
	buildBvh();
	buildWindingNumbers();
}

size_t TriangleMesh::numberOfTriangles() const
{
	return _pointIndices.size();
}

Triangle TriangleMesh::triangle(size_t i) const
{
	Triangle tri;
	for (int j = 0; j < 3; j++) 
	{
		if (j == 0)
		{
			tri.points[j] = _points[_pointIndices[i].x];
		}
		else if (j == 1)
		{
			tri.points[j] = _points[_pointIndices[i].y];
		}
		else
		{
			tri.points[j] = _points[_pointIndices[i].z];
		}
	}

	Vector3 n = tri.faceNormal();

	for (int j = 0; j < 3; j++) 
	{
		if (hasNormals()) 
		{
			if (j == 0)
			{
				tri.points[j] = _points[_pointIndices[i].x];
			}
			else if (j == 1)
			{
				tri.points[j] = _points[_pointIndices[i].y];
			}
			else
			{
				tri.points[j] = _points[_pointIndices[i].z];
			}
		}
		else 
		{
			tri.normals[j] = n;
		}
	}

	return tri;
}

TriangleMesh::Builder TriangleMesh::builder()
{
	return Builder();
}

Vector3 TriangleMesh::closestPointLocal(Vector3 otherPoint) const
{
	buildBvh();

	const auto distanceFunc = [this](const size_t& triIdx, const Vector3& pt)
	{
		Triangle tri = triangle(triIdx);
		return tri.closestDistance(pt);
	};

	const auto queryResult = _bvh.nearest(otherPoint, distanceFunc);
	return triangle(*queryResult.item).closestPoint(otherPoint);
}

double TriangleMesh::closestDistanceLocal(Vector3 otherPoint)
{
	buildBvh();

	const auto distanceFunc = [this](const size_t& triIdx, const Vector3& pt) 
	{
		Triangle tri = triangle(triIdx);
		return tri.closestDistance(pt);
	};

	const auto queryResult = _bvh.nearest(otherPoint, distanceFunc);
	return queryResult.distance;
}

BoundingBox TriangleMesh::boundingBoxLocal() const
{
	buildBvh();

	return _bvh.boundingBox();
}

Vector3 TriangleMesh::closestNormalLocal(const Vector3 & otherPoint) const
{
	buildBvh();

	const auto distanceFunc = [this](const size_t& triIdx, const Vector3& pt) 
	{
		Triangle tri = triangle(triIdx);
		return tri.closestDistance(pt);
	};

	const auto queryResult = _bvh.nearest(otherPoint, distanceFunc);
	return triangle(*queryResult.item).closestNormal(otherPoint);
}

bool TriangleMesh::isInsideLocal(Vector3 otherPoint)
{
	return fastWindingNumber(otherPoint, kDefaultFastWindingNumberAccuracy) > 0.5;
}

void TriangleMesh::buildBvh() const
{
	if (_bvhInvalidated) 
	{
		size_t nTris = numberOfTriangles();
		std::vector<size_t> ids(nTris);
		std::vector<BoundingBox> bounds(nTris);
		for (size_t i = 0; i < nTris; ++i)
		{
			ids[i] = i;
			bounds[i] = triangle(i).boundingBox();
		}
		_bvh.build(ids, bounds);
		_bvhInvalidated = false;
	}
}

void TriangleMesh::buildWindingNumbers() const
{
	if (_wnInvalidated) {
		buildBvh();

		size_t nNodes = _bvh.numberOfNodes();
		_wnAreaWeightedNormalSums.resize(nNodes);
		_wnAreaWeightedAvgPositions.resize(nNodes);

		const auto visitorFunc = [&](size_t nodeIndex,
			const WindingNumberGatherData& data) 
		{
			_wnAreaWeightedNormalSums[nodeIndex] = data.areaWeightedNormalSums;
			_wnAreaWeightedAvgPositions[nodeIndex] =
				data.areaWeightedPositionSums / data.areaSums;

		};
		const auto leafFunc = [&](size_t nodeIndex) -> WindingNumberGatherData 
		{
			WindingNumberGatherData result;

			auto iter = _bvh.itemOfNode(nodeIndex);

			Triangle tri = triangle(*iter);
			double area = tri.area();
			result.areaSums = area;
			result.areaWeightedNormalSums = tri.faceNormal() * area;
			result.areaWeightedPositionSums =
				((tri.points[0] + tri.points[1] + tri.points[2]) * area)/ 3.0;

			return result;
		};

		postOrderTraversal(_bvh, 0, visitorFunc, leafFunc,
			WindingNumberGatherData());

		_wnInvalidated = false;
	}
}

double TriangleMesh::windingNumber(const Vector3 & queryPoint, size_t triIndex) const
{	
	const Vector3& vi = _points[_pointIndices[triIndex].x];
	const Vector3& vj = _points[_pointIndices[triIndex].y];
	const Vector3& vk = _points[_pointIndices[triIndex].z];
	const Vector3 va = vi - queryPoint;
	const Vector3 vb = vj - queryPoint;
	const Vector3 vc = vk - queryPoint;
	const double a = va.length();
	const double b = vb.length();
	const double c = vc.length();

	const Matrix3x3 mat(va.x, vb.x, vc.x, va.y, vb.y, vc.y, va.z, vb.z, vc.z);
	const double det = mat.determinant();
	const double denom =
		a * b * c + va.dot(vb) * c + vb.dot(vc) * a + vc.dot(va) * b;

	const double solidAngle = 2.0 * std::atan2(det, denom);

	return solidAngle;
}

double TriangleMesh::fastWindingNumber(const Vector3 & queryPoint, double accuracy) const
{
	buildWindingNumbers();

	return fastWindingNumber(queryPoint, 0, accuracy);
}

double TriangleMesh::fastWindingNumber(const Vector3 & queryPoint, size_t rootNodeIndex, double accuracy) const
{
	const Vector3& treeP = _wnAreaWeightedAvgPositions[rootNodeIndex];
	const double qToP2 = queryPoint.distanceSquaredTo(treeP);

	const Vector3& treeN = _wnAreaWeightedNormalSums[rootNodeIndex];
	const BoundingBox& treeBound = _bvh.nodeBound(rootNodeIndex);
	Vector3 treeRVec;
	treeRVec.x = std::max((treeP - treeBound.lowerCorner).x, (treeBound.upperCorner - treeP).x);
	treeRVec.y = std::max((treeP - treeBound.lowerCorner).y, (treeBound.upperCorner - treeP).y);
	treeRVec.z = std::max((treeP - treeBound.lowerCorner).z, (treeBound.upperCorner - treeP).z);
	const double treeR = treeRVec.length();

	if (qToP2 > (accuracy * treeR)*(accuracy * treeR))
	{
		// Case: q is sufficiently far from all elements in tree
		// TODO: This is zero-th order approximation. Higher-order approximation
		return (treeP - queryPoint).dot(treeN) / (4.0 * kPiD * (std::sqrt(qToP2))*(std::sqrt(qToP2))*(std::sqrt(qToP2)));
	}
	else 
	{
		if (_bvh.isLeaf(rootNodeIndex)) 
		{
			// Case: q is nearby; use direct sum for tree’s elements
			auto iter = _bvh.itemOfNode(rootNodeIndex);
			return windingNumber(queryPoint, *iter) * (0.25 / kPiD);
		}
		else 
		{
			// Case: Recursive call
			const auto children = _bvh.children(rootNodeIndex);
			double wn = 0.0;
			wn += fastWindingNumber(queryPoint, children.first, accuracy);
			wn += fastWindingNumber(queryPoint, children.second, accuracy);
			return wn;
		}
	}
}

TriangleMesh::Builder & TriangleMesh::Builder::withIsNormalFlipped(bool isNormalFlipped)
{
	_isNormalFlipped = isNormalFlipped;
	return static_cast<TriangleMesh::Builder&>(*this);
}

TriangleMesh::Builder & TriangleMesh::Builder::withTranslation(const Vector3 & translation)
{
	_transform.setTranslation(translation);
	return static_cast<TriangleMesh::Builder&>(*this);
}

TriangleMesh::Builder & TriangleMesh::Builder::withOrientation(const Quaternion & orientation)
{
	_transform.setOrientation(orientation);
	return static_cast<TriangleMesh::Builder&>(*this);
}

TriangleMesh::Builder & TriangleMesh::Builder::withTransform(const Transform & transform)
{
	_transform = transform;
	return static_cast<TriangleMesh::Builder&>(*this);
}

TriangleMesh::Builder & TriangleMesh::Builder::withPoints(const PointArray & points)
{
	_points = points;
	return *this;
}

TriangleMesh::Builder & TriangleMesh::Builder::withNormals(const NormalArray & normals)
{
	_normals = normals;
	return *this;
}

TriangleMesh::Builder & TriangleMesh::Builder::withPointIndices(const IndexArray & pointIndices)
{
	_pointIndices = pointIndices;
	return *this;
}

TriangleMesh::Builder & TriangleMesh::Builder::withNormalIndices(const IndexArray & normalIndices)
{
	_normalIndices = normalIndices;
	return *this;
}

TriangleMesh TriangleMesh::Builder::build() const
{
	return TriangleMesh(_points, _normals, _pointIndices, _normalIndices, _transform, _isNormalFlipped);
}

TriangleMeshPtr TriangleMesh::Builder::makeShared() const
{
	return std::shared_ptr<TriangleMesh>(
		new TriangleMesh(_points, _normals, _pointIndices,
			_normalIndices, _transform,
			_isNormalFlipped),
		[](TriangleMesh* obj) { delete obj; });
}