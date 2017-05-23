#include "RayIntersection.h"
#include "Vector3.h"

#include <vector>

int main()
{
	std::vector<mk::Vector3> hits;

	mk::Vector3 boxCenter(0.0f, 0.0f, 0.0f);
	mk::Vector3 boxHalfLengths(1.0f, 2.0f, 3.0f);

	mk::Vector3 rayOrigin(3.0f, 0.0f, 0.0f);
	mk::Vector3 rayDirection(-1.0f, 0.0f, 0.0f);

	bool result = mk::RayAABoxIntersection(rayOrigin, rayDirection, boxCenter, boxHalfLengths, hits);

	mk::Vector3 sphereCenter(0.0f, 0.0f, 0.0f);
	result = mk::RaySphereIntersection(rayOrigin, rayDirection, sphereCenter, 4.0f, hits);

	return 0;
}