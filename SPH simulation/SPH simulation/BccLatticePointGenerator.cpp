#include "BccLatticePointGenerator.h"

void BccLatticePointGenerator::forEachPoint(const BoundingBox & boundingBox, double spacing, const std::function<bool(const Vector3&)>& callback) const
{
	double halfSpacing = spacing / 2.0;
	double boxWidth = boundingBox.width();
	double boxHeight = boundingBox.height();
	double boxDepth = boundingBox.depth();

	Vector3 pos;
	bool hasOffset = false;
	bool shouldQuit = false;
	for (int k = 0; k*halfSpacing <= boxDepth && !shouldQuit; ++k)
	{
		pos.z = k * halfSpacing + boundingBox.lowerCorner.z;

		double offset = (hasOffset) ? halfSpacing : 0.0;

		for (int j = 0; j*spacing + offset <= boxHeight && !shouldQuit; ++j)
		{
			pos.y = j * spacing + offset + boundingBox.lowerCorner.y;

			for (int i = 0; i *spacing + offset <= boxWidth; ++i)
			{
				pos.x = i * spacing + offset + boundingBox.lowerCorner.x;
				if (!callback(pos))
				{
					shouldQuit = true;
					break;
				}
			}
		}
		hasOffset = !hasOffset;
	}
}