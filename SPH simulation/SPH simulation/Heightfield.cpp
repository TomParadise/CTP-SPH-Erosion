#include "Heightfield.h"

Heightfield::Heightfield(std::vector<Vector3> points, bool isNormalFlipped, size_t resolutionX, size_t resolutionZ, BoundingBox maxRegion)
{
	_points = points;
	_isNormalFlipped = isNormalFlipped;
	_resolution_x = resolutionX;
	_resolution_z = resolutionZ;
	_maxRegion = maxRegion;
}

Vector3 Heightfield::closestPointLocal(Vector3 otherPoint) const
{
	Vector3 vertex1 = _points[std::floor(otherPoint.z)*_resolution_x + std::floor(otherPoint.x)] + Vector3(1,0,0);
	Vector3 vertex2;
	Vector3 vertex3;
	
	double _relativeXPos = vertex1.x - otherPoint.x;
	double _relativeZPos = otherPoint.z - vertex1.z;

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

	Vector3 normal = closestNormalLocal(otherPoint);

	double t = ((vertex1 - otherPoint).dot(normal))/((normal*-1).dot(normal));

	Vector3 point = otherPoint + (normal*-1) * t;

	return point;
}

double Heightfield::closestDistanceLocal(Vector3 otherPoint)
{
	return closestPointLocal(otherPoint).distanceTo(otherPoint);
}

BoundingBox Heightfield::boundingBoxLocal() const
{
	return _maxRegion;
}

Vector3 Heightfield::closestNormalLocal(const Vector3 & otherPoint) const
{
	Vector3 point = otherPoint;
	if (point.x > _resolution_x - 1)
	{
		point.x = _resolution_x;
	}
	if (point.z > _resolution_z - 1)
	{
		point.z = _resolution_z;
	}
	Vector3 vertex1 = _points[std::floor(point.z)*_resolution_x + std::floor(point.x)];
	vertex1.x += 1;
	Vector3 vertex2;
	Vector3 vertex3;
	double _relativeXPos = vertex1.x - point.x;
	double _relativeZPos = point.z - vertex1.z;

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

	Vector3 edge1 = vertex1 - vertex2;
	Vector3 edge2 = vertex1 - vertex3;

	return edge1.cross(edge2).normalized();

}

bool Heightfield::isInsideLocal(Vector3 otherPoint)
{
	return otherPoint.y < closestPointLocal(otherPoint).y;
}

Heightfield::Builder Heightfield::builder()
{
	return Builder();
}

void Heightfield::depositToNode(Vector3 pos, double amountToDeposit)
{
	// Add the sediment to the four vertices of the current node 
	// using bilinear interpolation
	// Deposition is not distributed over a radius (like erosion) 
	// so that it can fill small pits
	Vector3& vertex1 = _points[std::floor(pos.z)*_resolution_x + std::floor(pos.x)];
	Vector3& vertex2 = _points[std::floor(pos.z)*_resolution_x + std::floor(pos.x)+1];
	Vector3& vertex3 = _points[(std::floor(pos.z)+1)*_resolution_x + std::floor(pos.x)];
	Vector3& vertex4 = _points[(std::floor(pos.z) + 1)*_resolution_x + std::floor(pos.x)+1];

	vertex1.y += amountToDeposit * (1 - (pos.x - vertex1.x)) * (1 - (pos.z - vertex1.z));
	vertex2.y += amountToDeposit * ((pos.x - vertex1.x)) * (1 - (pos.z - vertex1.z));
	vertex3.y += amountToDeposit * (1 - (pos.x - vertex1.x)) * (pos.z - vertex1.z);
	vertex4.y += amountToDeposit * ((pos.x - vertex1.x)) * (pos.z - vertex1.z);
}

double Heightfield::erodeNode(Vector3 pos, double amountToErode)
{
	double erosionRadius = 2.0;
	int nodeindex = std::floor(pos.z)*_resolution_x + std::floor(pos.x);
	double sediment = 0;

	Vector3& vertex1 = _points[std::floor(pos.z)*_resolution_x + std::floor(pos.x)];
	Vector3& vertex2 = _points[std::floor(pos.z)*_resolution_x + std::floor(pos.x) + 1];
	Vector3& vertex3 = _points[(std::floor(pos.z) + 1)*_resolution_x + std::floor(pos.x)];
	Vector3& vertex4 = _points[(std::floor(pos.z) + 1)*_resolution_x + std::floor(pos.x) + 1];

	double _relativeXPos = pos.x - vertex1.x;
	double _relativeZPos = pos.z - vertex1.z;
	//the node the particle is in
	if (pos.distanceTo(vertex1) <= erosionRadius)
	{
		vertex1.y -= amountToErode * (1 - (pos.x - vertex1.x)) * (1 - (pos.z - vertex1.z));
		sediment += amountToErode * (1 - (pos.x - vertex1.x)) * (1 - (pos.z - vertex1.z));
	}
	if (pos.distanceTo(vertex2) <= erosionRadius)
	{
		vertex2.y -= amountToErode * (pos.x - vertex1.x) * (1 - (pos.z - vertex1.z));
		sediment += amountToErode * (pos.x - vertex1.x) * (1 - (pos.z - vertex1.z));
	}
	if (pos.distanceTo(vertex3) <= erosionRadius)
	{
		vertex3.y -= amountToErode * (1 - (pos.x - vertex1.x)) * (pos.z - vertex1.z);
		sediment += amountToErode * (1 - (pos.x - vertex1.x)) * (pos.z - vertex1.z);
	}
	if (pos.distanceTo(vertex4) <= erosionRadius)
	{
		vertex4.y -= amountToErode * (pos.x - vertex1.x) * (pos.z - vertex1.z);
		sediment += amountToErode * (pos.x - vertex1.x) * (pos.z - vertex1.z);
	}
	//the node to the west, if there is one
	if (pos.x > 1)
	{
		Vector3& vertex5 = _points[std::floor(pos.z)*_resolution_x + std::floor(pos.x) - 1];
		if (pos.distanceTo(vertex5) <= erosionRadius)
		{
			vertex5.y -= amountToErode * (1 - (pos.x - vertex1.x)) * (1 - (pos.z - vertex1.z));
			sediment += amountToErode * (1 - (pos.x - vertex1.x)) * (1 - (pos.z - vertex1.z));
		}
		if (pos.z < _resolution_z - 2)
		{
			Vector3& vertex6 = _points[(std::floor(pos.z) + 1)*_resolution_x + std::floor(pos.x) - 1];
			if (pos.distanceTo(vertex6) <= erosionRadius)
			{
				vertex6.y -= amountToErode * (1 - (pos.x - vertex1.x)) * (pos.z - vertex1.z);
				sediment += amountToErode * (1 - (pos.x - vertex1.x)) * (pos.z - vertex1.z);
			}
		}
	}
	//the node to the north, if there is one
	if (pos.z < _resolution_z - 3)
	{
		Vector3& vertex7 = _points[(std::floor(pos.z) + 2)*_resolution_x + std::floor(pos.x)];
		if (pos.distanceTo(vertex7) <= erosionRadius)
		{
			vertex7.y -= amountToErode * (1 - (pos.x - vertex1.x)) * (pos.z - vertex1.z);
			sediment += amountToErode * (1 - (pos.x - vertex1.x)) * (pos.z - vertex1.z);
		}
		if (pos.x < _resolution_x - 2)
		{
			Vector3& vertex8 = _points[(std::floor(pos.z) + 2)*_resolution_x + std::floor(pos.x) + 1];
			if (pos.distanceTo(vertex8) <= erosionRadius)
			{
				vertex8.y -= amountToErode * (pos.x - vertex1.x) * (pos.z - vertex1.z);
				sediment += amountToErode * (pos.x - vertex1.x) * (pos.z - vertex1.z);
			}
		}
	}
	//the node to the east, if there is one
	if (pos.x < _resolution_x - 3)
	{
		Vector3& vertex9 = _points[std::floor(pos.z)*_resolution_x + std::floor(pos.x) + 2];
		if (pos.distanceTo(vertex9) <= erosionRadius)
		{
			vertex9.y -= amountToErode * (pos.x - vertex1.x) * (1 - (pos.z - vertex1.z));
			sediment += amountToErode * (pos.x - vertex1.x) * (1 - (pos.z - vertex1.z));
		}
		if (pos.z < _resolution_z - 2)
		{
			Vector3& vertex10 = _points[(std::floor(pos.z) + 1)*_resolution_x + std::floor(pos.x) + 2];
			if (pos.distanceTo(vertex10) <= erosionRadius)
			{
				vertex10.y -= amountToErode * (pos.x - vertex1.x) * (pos.z - vertex1.z);
				sediment += amountToErode * (pos.x - vertex1.x) * (pos.z - vertex1.z);
			}
		}
	}
	//the node to the south, if there is one
	if (pos.z > 1)
	{
		Vector3& vertex11 = _points[(std::floor(pos.z) - 1)*_resolution_x + std::floor(pos.x)];
		if (pos.distanceTo(vertex11) <= erosionRadius)
		{
			vertex11.y -= amountToErode * (1 - (pos.x - vertex1.x)) * (1 - (pos.z - vertex1.z));
			sediment += amountToErode * (1 - (pos.x - vertex1.x)) * (1 - (pos.z - vertex1.z));
		}
		if (pos.x < _resolution_x - 2)
		{
			Vector3& vertex12 = _points[(std::floor(pos.z) - 1)*_resolution_x + std::floor(pos.x) + 1];
			if (pos.distanceTo(vertex12) <= erosionRadius)
			{
				vertex12.y -= amountToErode * (pos.x - vertex1.x) * (1 - (pos.z - vertex1.z));
				sediment += amountToErode * (pos.x - vertex1.x) * (1 - (pos.z - vertex1.z));
			}
		}
	}
	return sediment;
}

std::vector<Vector3> Heightfield::getVertices()
{
	return _points;
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
	return Heightfield(_points, _isNormalFlipped, _resolution_x, _resolution_z, _maxRegion);
}

HeightfieldPtr Heightfield::Builder::makeShared() const
{
	return std::shared_ptr<Heightfield>(
		new Heightfield(_points, _isNormalFlipped, _resolution_x, _resolution_z, _maxRegion),
		[](Heightfield* obj) { delete obj; });
}

Heightfield::Builder & Heightfield::Builder::withBox(BoundingBox maxRegion)
{
	_maxRegion = maxRegion;
	return *this;
}
