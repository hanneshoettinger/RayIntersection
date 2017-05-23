#pragma once

#include "Vector3.h"
#include <vector>

namespace mk
{
	static Vector3 ProjectPointOntoRay(const Vector3& rayOrigin, const Vector3& rayDirection, const Vector3& point)
	{
		Vector3 originToPoint = point - rayOrigin;

		// assume the direction is not normalized
		float dist = (rayDirection.Dot(originToPoint)) / rayDirection.Length();

		Vector3 result = rayDirection * dist;
		result = rayOrigin + result;

		return result;
	}

	static bool RaySphereIntersection(const Vector3& rayOrigin, const Vector3& rayDirection, const Vector3& sphereCenter, float radius, std::vector<Vector3>& hits)
	{
		// make sure the direction is a unit vector
		Vector3 direction = Vector3::Normalize(rayDirection);

		Vector3 originToCenter = sphereCenter - rayOrigin;

		// check whether the center of the sphere is behind the ray origin
		if (originToCenter.Dot(direction) < 0.0f)
		{
			// the sphere center is behind the ray -> intersection is only possible if the ray is within the sphere

			float distance = originToCenter.Length();
			if (distance > radius)
			{
				// ray origin is outside the sphere
				return false;
			}
			else if (distance > (radius - 0.000001f) && distance < (radius + 0.000001f))
			{
				// ray origin is on the sphere
				hits.push_back(rayOrigin);
				return true;
			}
			else
			{
				// get the projection point from the sphere center onto the ray
				Vector3 projected = ProjectPointOntoRay(rayOrigin, direction, sphereCenter);

				// get the intersection point
				float lengthProjCenter = (projected - sphereCenter).Length();
				float dist = sqrtf((radius * radius) + (lengthProjCenter * lengthProjCenter));

				float lengthOriginIntersection = dist - (projected - rayOrigin).Length();

				Vector3 hit = rayOrigin + (direction * lengthOriginIntersection);
				hits.push_back(hit);

				return true;
			}

		}
		else
		{
			// the sphere center is in front of the ray

			Vector3 projected = ProjectPointOntoRay(rayOrigin, direction, sphereCenter);

			float lengthProjCenter = (sphereCenter - projected).Length();
			if (lengthProjCenter > radius)
			{
				// the projection point is outside the sphere -> no intersection
				return false;
			}
			else if (lengthProjCenter > (radius - 0.000001f) && lengthProjCenter < (radius + 0.000001f))
			{
				// the projection point is on the sphere
				hits.push_back(projected);
				return true;
			}

			float lengthProjIntersection1 = sqrtf((radius * radius) + (lengthProjCenter * lengthProjCenter));

			// check whether the ray origin is within the sphere
			if (originToCenter.Length() < radius)
			{
				// there is only one intersection
				float lengthOriginIntersection = (projected - rayOrigin).Length() + lengthProjIntersection1;

				Vector3 hit = rayOrigin + (direction * lengthOriginIntersection);
				hits.push_back(hit);

				return true;
			}
			else
			{
				// there are two intersections
				// get the first intersection
				float lengthProjOrigin = (projected - rayOrigin).Length();
				float lengthOriginIntersection = lengthProjOrigin - lengthProjIntersection1;
				Vector3 hit = rayOrigin + (direction * lengthOriginIntersection);
				hits.push_back(hit);

				// get the second intersection point
				lengthOriginIntersection = lengthProjOrigin + lengthProjIntersection1;
				hit = rayOrigin + (direction * lengthOriginIntersection);
				hits.push_back(hit);

				return true;
			}

		}
	}

	static bool RayAABoxIntersection(const Vector3& rayOrigin, const Vector3& rayDirection, const Vector3& boxCenter, const Vector3&  boxHalfLengths, std::vector<Vector3>& hits)
	{
		// get the min and maz values of the box
		Vector3 boxLimits[2] =
		{	boxCenter - boxHalfLengths,
			boxCenter + boxHalfLengths
		};

		// make sure the direction is normalized
		Vector3 direction = Vector3::Normalize(rayDirection);

		// get the inverse direction
		Vector3 invDirecton(	1.0f / direction.x,
								1.0f / direction.y,
								1.0f / direction.z);
		
		// get the direction signs
		int sign[3] = 
		{	invDirecton.x < 0.0f,
			invDirecton.y < 0.0f,
			invDirecton.z < 0.0f
		};

		// handle the x and y slabs
		float min = (boxLimits[sign[0]].x - rayOrigin.x) * invDirecton.x;
		float max = (boxLimits[1 - sign[0]].x - rayOrigin.x) * invDirecton.x;
		float minY = (boxLimits[sign[1]].y - rayOrigin.y) * invDirecton.y;
		float maxY = (boxLimits[1 - sign[1]].y - rayOrigin.y) * invDirecton.y;

		// check whether the ray misses the slab
		if (min > maxY || max < minY)
		{
			return false;
		}

		// adjust the intersection values
		if (minY > min)
		{
			min = minY;
		}

		if (maxY < max)
		{
			max = maxY;
		}

		// handle the z slab
		float minZ = (boxLimits[sign[2]].z - rayOrigin.z) * invDirecton.z;
		float maxZ = (boxLimits[1 - sign[2]].z - rayOrigin.z) * invDirecton.z;

		// check whether the ray misses the slab
		if (min > maxZ || max < minZ)
		{
			return false;
		}

		// adjust the intersection values
		if (minZ > min)
		{
			min = minZ;
		}

		if (maxZ < max)
		{
			max = maxZ;
		}

		bool result = false;

		// check whether the first intersection is in front of the ray origin
		Vector3 hit;
		if (min >= 0.0f)
		{
			hit = rayOrigin + (direction * min);
			hits.push_back(hit);

			result = true;
		}

		// check whether the second intersection is in front of the ray origin
		if (max >= 0.0f && min != max)
		{
			hit = rayOrigin + (direction * max);
			hits.push_back(hit);

			result = true;
		}

		return result;
	}

	static bool RayOBoxIntersection(const Vector3& rayOrigin, const Vector3& rayDirection, const Vector3& boxCenter, const Vector3&  boxHalfLengths, const Vector3& orientationAxis, float angle, std::vector<Vector3>& hits)
	{
		// turnt the oriented box into a axis aligned box
		Vector3 origin = Vector3::Rotate(rayOrigin, orientationAxis, -angle);
		Vector3 direction = Vector3::Rotate(rayDirection, orientationAxis, -angle);

		Vector3 center = Vector3::Rotate(rayDirection, orientationAxis, -angle);

		// perform the intersection with the axis aligned box
		bool result = RayAABoxIntersection(origin, direction, center, boxHalfLengths, hits);

		// rotate the hits back
		for (size_t i = 0; i < hits.size(); ++i)
		{
			hits[i].Rotate(orientationAxis, angle);
		}

		return result;
	}
}

namespace jk
{
	float kEpsilon = 0.001f;
	static bool rayTriangleIntersect( const mk::Vector3 &rayOrigin, const mk::Vector3 &rayDirection,
		const mk::Vector3 &v0, const mk::Vector3 &v1, const mk::Vector3 &v2,
		float &t)
	{
		// compute plane's normal
		mk::Vector3 v0v1 = v1 - v0;
		mk::Vector3 v0v2 = v2 - v0;
		// no need to normalize
		mk::Vector3 N = v0v1.Cross(v0v2); // N 
		float area2 = N.Length();


		// Step 1: finding P

		// check if ray and plane are parallel ?
		float NdotRayDirection = N.Dot(rayDirection);
		if (fabs(NdotRayDirection) < kEpsilon) // almost 0 
			return false; // they are parallel so they don't intersect ! 

		// compute d parameter using equation 2
		float d = N.Dot(v0);

		// compute t (equation 3)
		t = (N.Dot(rayOrigin) + d) / NdotRayDirection;
		// check if the triangle is in behind the ray
		if (t < 0) return false; // the triangle is behind 

		// compute the intersection point using equation 1
		mk::Vector3 P = rayOrigin + rayDirection*t;

		// Step 2: inside-outside test
		mk::Vector3 C; // vector perpendicular to triangle's plane 

		// edge 0
		mk::Vector3 edge0 = v1 - v0;
		mk::Vector3 vp0 = P - v0;
		C = edge0.Cross(vp0);
		if (N.Dot(C) < 0) return false; // P is on the right side 

		// edge 1
		mk::Vector3 edge1 = v2 - v1;
		mk::Vector3 vp1 = P - v1;
		C = edge1.Cross(vp1);
		if (N.Dot(C) < 0)  return false; // P is on the right side 

		// edge 2
		mk::Vector3 edge2 = v0 - v2;
		mk::Vector3 vp2 = P - v2;
		C = edge2.Cross(vp2);
		if (N.Dot(C) < 0) return false; // P is on the right side; 

		return true; // this ray hits the triangle 
	}
	static bool rayTriangleIntersectMT( 
    const mk::Vector3 &orig, const mk::Vector3 &dir,
    const mk::Vector3 &v0, const mk::Vector3 &v1, const mk::Vector3 &v2,
    float &t, float &u, float &v,bool bBackFaceCulling) 
	{ 
		mk::Vector3 v0v1 = v1 - v0;
		mk::Vector3 v0v2 = v2 - v0;
		mk::Vector3 pvec = dir.Cross(v0v2);
		float det = v0v1.Dot(pvec); 
		if (bBackFaceCulling)
		{
			// if the determinant is negative the triangle is backfacing
			// if the determinant is close to 0, the ray misses the triangle
			if (det < kEpsilon) return false;
		}
		else
		{
			// ray and triangle are parallel if det is close to 0
			if (fabs(det) < kEpsilon) return false;
		}
		float invDet = 1 / det; 
 
		mk::Vector3 tvec = orig - v0;
		u = tvec.Dot(pvec) * invDet; 
		if (u < 0 || u > 1) return false; 
 
		mk::Vector3 qvec = tvec.Cross(v0v1);
		v = dir.Dot(qvec) * invDet; 
		if (v < 0 || u + v > 1) return false; 
 
		t = v0v2.Dot(qvec) * invDet; 
 
		return true; 

	} 
	static bool rayTriangleIntersectB(
		const mk::Vector3 &orig, const mk::Vector3 &dir,
		const mk::Vector3 &v0, const mk::Vector3 &v1, const mk::Vector3 &v2,
		float &t, float &u, float &v)
	{
		// compute plane's normal
		mk::Vector3 v0v1 = v1 - v0;
		mk::Vector3 v0v2 = v2 - v0;
		// no need to normalize
		mk::Vector3 N = v0v1.Cross(v0v2); // N 
		float denom = N.Dot(N);

		// Step 1: finding P

		// check if ray and plane are parallel ?
		float NdotRayDirection = N.Dot(dir);
		if (fabs(NdotRayDirection) < kEpsilon) // almost 0 
			return false; // they are parallel so they don't intersect ! 

						  // compute d parameter using equation 2
		float d = N.Dot(v0);

		// compute t (equation 3)
		t = (N.Dot(orig) + d) / NdotRayDirection;
		// check if the triangle is in behind the ray
		if (t < 0) return false; // the triangle is behind 

								 // compute the intersection point using equation 1
		mk::Vector3 P = orig + dir*t;

		// Step 2: inside-outside test
		mk::Vector3 C; // vector perpendicular to triangle's plane 

		// edge 0
		mk::Vector3 edge0 = v1 - v0;
		mk::Vector3 vp0 = P - v0;
		C = edge0.Cross(vp0);
		if (N.Dot(C) < 0) return false; // P is on the right side 

	   // edge 1
		mk::Vector3 edge1 = v2 - v1;
		mk::Vector3 vp1 = P - v1;
		C = edge1.Cross(vp1);
		if ((u = N.Dot(C)) < 0)  return false; // P is on the right side 

		// edge 2
		mk::Vector3 edge2 = v0 - v2;
		mk::Vector3 vp2 = P - v2;
		C = edge2.Cross(vp2);
		if ((v = N.Dot(C)) < 0) return false; // P is on the right side; 

		u /= denom;
		v /= denom;

		return true; // this ray hits the triangle 
	}
}

namespace hh
{
	// PointInPolygon Test -> vert: number of vertices; vertx & verty: vertices of polygon; testx & testy: point on plane to test against polygon
	static bool PointInPolygon(int nvert, float *vertx, float *verty, float testx, float testy)
	{
		int i, j;
		bool inside = false;
		for (i = 0, j = nvert - 1; i < nvert; j = i++) {
			if (((verty[i] > testy) != (verty[j] > testy)) &&
				(testx < (vertx[j] - vertx[i]) * (testy - verty[i]) / (verty[j] - verty[i]) + vertx[i]))
				inside = !inside;
		}
		return inside;
	}
}