#include "Transform.h"

Transform::Transform()
{
	_orientationMat3 = _orientation.matrix3();
	_inverseOrientationMat3 = _orientation.inverse().matrix3();
}

Transform::Transform(const Vector3 & translation, const Quaternion& orientation)
{
	_translation = translation;
	setOrientation(orientation);
}

Transform::Transform(const Transform & transform)
{
	_translation = transform.translation();
	_orientation = transform.orientation();
	_orientationMat3 = transform.orientationMatrix();
	_inverseOrientationMat3 = transform.inverseOrientationMatrix();
}

const Vector3& Transform::translation() const
{
	return _translation;
}

void Transform::setTranslation(const Vector3 & translation)
{
	_translation = translation;
}

Vector3 Transform::toLocal(Vector3 & pointInWorld)
{
	return _inverseOrientationMat3.vectorMultiply((pointInWorld.vectorSubtract(_translation)));
}

const Quaternion & Transform::orientation() const
{
	return _orientation;
}

void Transform::setOrientation(const Quaternion & orientation)
{
	_orientation = orientation;
	_orientationMat3 = orientation.matrix3();
	_inverseOrientationMat3 = orientation.inverse().matrix3();
}

Vector3 Transform::translation()
{
	return _translation;
}

Quaternion Transform::orientation()
{
	return _orientation;
}

Matrix3x3 Transform::orientationMatrix() const
{
	return _orientationMat3;
}

Matrix3x3 Transform::inverseOrientationMatrix() const
{
	return _inverseOrientationMat3;
}

Vector3 Transform::toWorld(Vector3 pointInLocal)
{
	return (_orientationMat3.vectorMultiply(pointInLocal)).vectorAdd(_translation);
}

BoundingBox Transform::toWorld(const BoundingBox & bboxInLocal)
{
	BoundingBox bboxInWorld;
	for (int i = 0; i < 8; ++i) {
		auto cornerInWorld = toWorld(bboxInLocal.corner(i));

		bboxInWorld.lowerCorner = Vector3(
			std::min(bboxInWorld.lowerCorner.x, cornerInWorld.x),
			std::min(bboxInWorld.lowerCorner.y, cornerInWorld.y),
			std::min(bboxInWorld.lowerCorner.z, cornerInWorld.z));
		bboxInWorld.upperCorner = Vector3(
			std::max(bboxInWorld.upperCorner.x, cornerInWorld.x),
			std::max(bboxInWorld.upperCorner.y, cornerInWorld.y),
			std::max(bboxInWorld.upperCorner.z, cornerInWorld.z));
	}
	return bboxInWorld;
}

Vector3 Transform::toWorldDirection(Vector3 & dirInLocal)
{
	return _orientationMat3.vectorMultiply(dirInLocal);
}
