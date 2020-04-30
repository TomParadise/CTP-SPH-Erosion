#include "Box.h"

Box::Box(const Transform & transform, bool isNormalFlipped)
: Surface(transform, isNormalFlipped) {}

Box::Box(const Vector3 & lowerCorner, const Vector3 & upperCorner, const Transform & transform, bool isNormalFlipped)
: Box(BoundingBox(lowerCorner, upperCorner), transform,isNormalFlipped) {}

Box::Box(const BoundingBox & boundingBox, const Transform & transform, bool isNormalFlipped)
: Surface(transform, isNormalFlipped), bound(boundingBox) {}

Box::Builder Box::builder()
{
	return Builder();
}

void Box::depositToNode(Vector3 pos, double amountToDeposit)
{
}

double Box::erodeNode(Vector3 pos, double amountToErode)
{
	return 0.0;
}

std::vector<Vector3> Box::getVertices()
{
	return std::vector<Vector3>();
}

Vector3 Box::closestPointLocal(Vector3 otherPoint) const
{
	if (bound.contains(otherPoint)) 
	{
		Plane planes[6] = 
		{ 
			Plane(Vector3(1, 0, 0), bound.upperCorner),
			Plane(Vector3(0, 1, 0), bound.upperCorner),
			Plane(Vector3(0, 0, 1), bound.upperCorner),
			Plane(Vector3(-1, 0, 0), bound.lowerCorner),
			Plane(Vector3(0, -1, 0), bound.lowerCorner),
			Plane(Vector3(0, 0, -1), bound.lowerCorner) 
		};

		Vector3 result = planes[0].closestPoint(otherPoint);
		double distanceSquared = result.distanceSquaredTo(otherPoint);

		for (int i = 1; i < 6; ++i)
		{
			Vector3 localResult = planes[i].closestPoint(otherPoint);
			double localDistanceSquared =
				localResult.distanceSquaredTo(otherPoint);

			if (localDistanceSquared < distanceSquared)
			{
				result = localResult;
				distanceSquared = localDistanceSquared;
			}
		}

		return result;
	}
	else 
	{
		otherPoint = Vector3(
			std::max(otherPoint.x, bound.lowerCorner.x),
			std::max(otherPoint.y, bound.lowerCorner.y),
			std::max(otherPoint.z, bound.lowerCorner.z));
		otherPoint = Vector3(
			std::min(otherPoint.x, bound.upperCorner.x),
			std::min(otherPoint.y, bound.upperCorner.y),
			std::min(otherPoint.z, bound.upperCorner.z));
		return otherPoint;
	}
}

BoundingBox Box::boundingBoxLocal() const
{
	return bound;
}

Vector3 Box::closestNormalLocal(const Vector3& otherPoint) const
{
	Plane planes[6] = { Plane(Vector3(1, 0, 0), bound.upperCorner),
					   Plane(Vector3(0, 1, 0), bound.upperCorner),
					   Plane(Vector3(0, 0, 1), bound.upperCorner),
					   Plane(Vector3(-1, 0, 0), bound.lowerCorner),
					   Plane(Vector3(0, -1, 0), bound.lowerCorner),
					   Plane(Vector3(0, 0, -1), bound.lowerCorner) };
	if (bound.contains(otherPoint)) 
	{
		Vector3 closestNormal = planes[0].normal;
		Vector3 closestPoint = planes[0].closestPoint(otherPoint);
		double minDistanceSquared = (closestPoint-otherPoint).lengthSquared();

		for (int i = 1; i < 6; ++i) 
		{
			Vector3 localClosestPoint = planes[i].closestPoint(otherPoint);
			double localDistanceSquared =
				(localClosestPoint-otherPoint).lengthSquared();

			if (localDistanceSquared < minDistanceSquared)
			{
				closestNormal = planes[i].normal;
				minDistanceSquared = localDistanceSquared;
			}
		}

		return closestNormal;
	}
	else 
	{
		Vector3 closestPoint = Vector3(
				std::max(otherPoint.x, bound.lowerCorner.x),
				std::max(otherPoint.y, bound.lowerCorner.y),
				std::max(otherPoint.z, bound.lowerCorner.z));
		closestPoint = Vector3(
			std::min(closestPoint.x, bound.upperCorner.x),
			std::min(closestPoint.y, bound.upperCorner.y),
			std::min(closestPoint.z, bound.upperCorner.z));
		Vector3 closestPointToInputPoint = otherPoint;
		closestPointToInputPoint -= closestPoint;
		Vector3 closestNormal = planes[0].normal;
		double maxCosineAngle = closestNormal.dot(closestPointToInputPoint);

		for (int i = 1; i < 6; ++i) 
		{
			double cosineAngle = planes[i].normal.dot(closestPointToInputPoint);

			if (cosineAngle > maxCosineAngle)
			{
				closestNormal = planes[i].normal;
				maxCosineAngle = cosineAngle;
			}
		}

		return closestNormal;
	}
}

Box::Builder & Box::Builder::withIsNormalFlipped(bool isNormalFlipped)
{
	_isNormalFlipped = isNormalFlipped;
	return static_cast<Box::Builder&>(*this);
}

Box::Builder & Box::Builder::withTranslation(const Vector3 & translation)
{
	_transform.setTranslation(translation);
	return static_cast<Box::Builder&>(*this);
}

Box::Builder & Box::Builder::withOrientation(const Quaternion & orientation)
{
	_transform.setOrientation(orientation);
	return static_cast<Box::Builder&>(*this);
}

Box::Builder & Box::Builder::withTransform(const Transform & transform)
{
	_transform = transform;
	return static_cast<Box::Builder&>(*this);
}

Box::Builder & Box::Builder::withLowerCorner(const Vector3 & pt)
{
	_lowerCorner = pt;
	return *this;
}

Box::Builder & Box::Builder::withUpperCorner(const Vector3 & pt)
{
	_upperCorner = pt;
	return *this;
}

Box::Builder & Box::Builder::withBoundingBox(const BoundingBox & bbox)
{
	_lowerCorner = bbox.lowerCorner;
	_upperCorner = bbox.upperCorner;
	return *this;
}

Box Box::Builder::build() const
{
	return Box(_lowerCorner, _upperCorner, _transform, _isNormalFlipped);
}

BoxPtr Box::Builder::makeShared() const
{
	return std::shared_ptr<Box>(
		new Box(_lowerCorner, _upperCorner, _transform, _isNormalFlipped),
		[](Box* obj) { delete obj; });
}