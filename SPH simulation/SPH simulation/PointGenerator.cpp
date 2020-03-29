#include "PointGenerator.h"



PointGenerator::PointGenerator()
{
}


PointGenerator::~PointGenerator()
{
}

void PointGenerator::generate(const BoundingBox & boundingBox, double spacing, std::vector<Vector3>* points) const
{
	forEachPoint(boundingBox, spacing, [&points](const Vector3& point)
	{		
		points->push_back(point);
		return true;
	});
}
