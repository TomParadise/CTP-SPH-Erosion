#include "Heightfield.h"

Heightfield::Heightfield(std::vector<Vector3> points, bool isNormalFlipped, size_t resolutionX, size_t resolutionZ)
{
	_points = points;
	_isNormalFlipped = isNormalFlipped;
	_resolution_x = resolutionX;
	_resolution_z = resolutionZ;
}

Vector3 Heightfield::closestPointLocal(Vector3 otherPoint) const
{
	if (otherPoint.z < 0)
	{
		otherPoint.z = 0;
	}
	else if (otherPoint.z > _resolution_z-1)
	{
		otherPoint.z = _resolution_z-1;
	}
	if (otherPoint.x < 0)
	{
		otherPoint.x = 0;
	}
	else if (otherPoint.x > _resolution_x-1)
	{
		otherPoint.x = _resolution_x-1;
	}
	Vector3 vertex1 = _points[std::floor(otherPoint.z)*_resolution_x + std::floor(otherPoint.x)] + Vector3(0.5,0,0.5);
	Vector3 vertex2;
	Vector3 vertex3;
	
	float _relativeXPos = otherPoint.x - vertex1.x;
	float _relativeZPos = otherPoint.z - vertex1.z;

	if (_relativeXPos >= _relativeZPos)
	{
		vertex1 = _points[std::floor(otherPoint.z)*_resolution_x + std::floor(otherPoint.x)];
		vertex2 = _points[(std::floor(otherPoint.z) + 1)*_resolution_x + std::floor(otherPoint.x)];
		vertex3 = _points[std::floor(otherPoint.z)*_resolution_x + std::floor(otherPoint.x) + 1];
	}
	else
	{
		vertex1 = _points[std::floor(otherPoint.z)*_resolution_x + std::floor(otherPoint.x) + 1];
		vertex2 = _points[(std::floor(otherPoint.z) + 1)*_resolution_x + std::floor(otherPoint.x)];
		vertex3 = _points[(std::floor(otherPoint.z) + 1)*_resolution_x + std::floor(otherPoint.x) + 1];
	}

	float lambda1 = ((vertex2.z - vertex3.z)*(otherPoint.x - vertex3.x) + (vertex3.x - vertex2.x)*(otherPoint.z - vertex3.z)) / 
					((vertex2.z-vertex3.z)*(vertex1.x-vertex3.x) + (vertex3.x-vertex2.x)*(vertex1.z-vertex3.z));
	float lambda2 = ((vertex3.z - vertex1.z)*(otherPoint.x - vertex3.x) + (vertex1.x - vertex3.x)*(otherPoint.z - vertex3.z)) /
					((vertex2.z - vertex3.z)*(vertex1.x - vertex3.x) + (vertex3.x - vertex2.x)*(vertex1.z - vertex3.z));
	float lambda3 = 1 - lambda1 - lambda2;

	float height = lambda1 * vertex1.y + lambda2 * vertex2.y + lambda3 * vertex3.y;

	return Vector3(otherPoint.x, height, otherPoint.z);
}

double Heightfield::closestDistanceLocal(Vector3 otherPoint)
{
	return closestPointLocal(otherPoint).distanceTo(otherPoint);
}

BoundingBox Heightfield::boundingBoxLocal() const
{
	return BoundingBox();
}

Vector3 Heightfield::closestNormalLocal(const Vector3 & otherPoint) const
{
	Vector3 point = otherPoint;
	if (otherPoint.z < 0)
	{
		point.z = 0;
	}
	else if (otherPoint.z > _resolution_z-1)
	{
		point.z = _resolution_z-1;
	}
	if (otherPoint.x < 0)
	{
		point.x = 0;
	}
	else if (otherPoint.x > _resolution_x-1)
	{
		point.x = _resolution_x-1;
	}
	Vector3 vertex1 = _points[std::floor(point.z)*_resolution_x + std::floor(point.x)] + Vector3(0.5, 0, 0.5);
	Vector3 vertex2;
	Vector3 vertex3;
	float _relativeXPos = point.x - vertex1.x;
	float _relativeZPos = point.z - vertex1.z;

	if (_relativeXPos >= _relativeZPos)
	{
		vertex1 = _points[std::floor(point.z)*_resolution_x + std::floor(point.x)];
		vertex2 = _points[(std::floor(point.z) + 1)*_resolution_x + std::floor(point.x)];
		vertex3 = _points[std::floor(point.z)*_resolution_x + std::floor(point.x) + 1];
	}
	else
	{
		vertex1 = _points[std::floor(point.z)*_resolution_x + std::floor(point.x) + 1];
		vertex2 = _points[(std::floor(point.z) + 1)*_resolution_x + std::floor(point.x)];
		vertex3 = _points[(std::floor(point.z) + 1)*_resolution_x + std::floor(point.x) + 1];
	}

	Vector3 edge1 = vertex2 - vertex1;
	Vector3 edge2 = vertex3 - vertex1;
	Vector3 norm = edge1.cross(edge2);
	return edge1.cross(edge2);

}

bool Heightfield::isInsideLocal(Vector3 otherPoint)
{
	return otherPoint.y < closestPointLocal(otherPoint).y;
}

Heightfield::Builder Heightfield::builder()
{
	return Builder();
}

Heightfield::Builder & Heightfield::Builder::withIsNormalFlipped(bool isNormalFlipped)
{
	_isNormalFlipped = isNormalFlipped;
	return *this;
}

Heightfield::Builder & Heightfield::Builder::withPoints(const std::vector<Vector3>& points)
{
	_points = points;
	return *this;
}

Heightfield::Builder & Heightfield::Builder::withResolution(size_t resolutionX, size_t resolutionZ)
{
	_resolution_x = resolutionX;
	_resolution_z = resolutionZ;
	return *this;
}

Heightfield Heightfield::Builder::build() const
{
	return Heightfield(_points, _isNormalFlipped, _resolution_x, _resolution_z);
}

HeightfieldPtr Heightfield::Builder::makeShared() const
{
	return std::shared_ptr<Heightfield>(
		new Heightfield(_points, _isNormalFlipped, _resolution_x, _resolution_z),
		[](Heightfield* obj) { delete obj; });
}