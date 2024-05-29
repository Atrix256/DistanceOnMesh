#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <random>
#include <vector>
#include <unordered_set>

#include "vecmath.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define SEED() 0
#define NUM_TRIANGLES() 40
#define IMAGE_SIZE() 2048


std::vector<Vec2> g_vertices;
std::vector<IVec3> g_indices;

std::mt19937 GetRNG()
{
	uint32_t seed;
	if (SEED() < 0)
	{
		std::random_device rd;
		seed = rd();
	}
	else
	{
		seed = SEED();
	}

	std::mt19937 rng(seed);
	return rng;
}

template <class T>
inline void hash_combine(std::size_t& seed, const T& v)
{
	std::hash<T> hasher;
	seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

template <size_t N>
struct IVecHash
{
	size_t operator()(const IVec<N>& vec) const
	{
		size_t ret = 0;
		for (int i : vec)
			hash_combine(ret, std::hash<int>()(i));
		return ret;
	}
};

template <size_t N>
Vec<N> RandomVec(std::mt19937& rng)
{
	std::uniform_real_distribution<float> dist(0.0f, 1.0f);
	Vec<N> ret;
	for (float& f : ret)
		f = dist(rng);
	return ret;
}

void MakeMesh()
{
	std::mt19937 rng = GetRNG();

	// Make the first triangle
	g_vertices.resize(3);
	g_vertices[0] = RandomVec<2>(rng);
	g_vertices[1] = RandomVec<2>(rng);
	g_vertices[2] = RandomVec<2>(rng);

	g_indices.resize(1);
	g_indices[0] = IVec3{ 0, 1, 2 };

	// keep a list of which edges are open and in need of a point
	std::unordered_set<IVec2, IVecHash<2>> openEdges;
	openEdges.insert(IVec2{ 0, 1 });
	openEdges.insert(IVec2{ 1, 2 });
	openEdges.insert(IVec2{ 2, 0 });

	// Make the rest of the triangles
	while (g_indices.size() < NUM_TRIANGLES())
	{
		// generate a new point for the mesh
		Vec2 newPoint = RandomVec<2>(rng);

		// If this point is contained by any of the triangles, don't use it
		{
			bool inTriangle = false;
			for (const IVec3& triangle : g_indices)
			{
				inTriangle = PointInTriangle(g_vertices[triangle[0]], g_vertices[triangle[1]], g_vertices[triangle[2]], newPoint);
				if (inTriangle)
					break;
			}
			if (inTriangle)
				continue;
		}

		// Else it's outside existing triangles. find which open edge it is nearest to.
		IVec2 nearestEdge;
		{
			float nearestEdgeDistance;
			bool first = true;
			for (const IVec2& edge : openEdges)
			{
				float distance = DistanceFromLineToPoint(g_vertices[edge[0]], g_vertices[edge[1]], newPoint);
				if (first || distance < nearestEdgeDistance)
				{
					first = false;
					nearestEdgeDistance = distance;
					nearestEdge = edge;
				}
			}
		}

		// If the 2 new lines intersect any existing triangle, ignore this new triangle
		// NOTE: we test the same edge multiple times. could fix that.
		{
			bool intersects = false;
			const Vec2& A = g_vertices[nearestEdge[0]];
			const Vec2& B = g_vertices[nearestEdge[1]];
			for (const IVec3& triangle : g_indices)
			{
				const Vec2& triA = g_vertices[triangle[0]];
				const Vec2& triB = g_vertices[triangle[1]];
				const Vec2& triC = g_vertices[triangle[2]];
				if (LinesIntersect(triA, triB, A, newPoint) ||
					LinesIntersect(triA, triB, B, newPoint) ||
					LinesIntersect(triB, triC, A, newPoint) ||
					LinesIntersect(triB, triC, B, newPoint) ||
					LinesIntersect(triC, triA, A, newPoint) ||
					LinesIntersect(triC, triA, B, newPoint))
				{
					intersects = true;
					break;
				}
			}

			if (intersects)
				continue;

			// TODO: handle when the points are the same point. it will sometimes say intersect and sometimes not due to numerical stuff
		}

		// add new triangle
		g_vertices.push_back(newPoint);
		g_indices.push_back(IVec3{ nearestEdge[0], nearestEdge[1], (int)g_vertices.size() - 1 });		

		// remove the open edge we just used
		openEdges.erase(IVec2{ nearestEdge[0], nearestEdge[1] });

		// add the 2 new open edges
		openEdges.insert(IVec2{ nearestEdge[0], (int)g_vertices.size() - 1 });
		openEdges.insert(IVec2{ nearestEdge[1], (int)g_vertices.size() - 1 });

		// TODO: make sure the points are in clockwise order?

		int ijkl = 0;
	}

	// TODO: draw out the result
	std::vector<unsigned char> pixels(IMAGE_SIZE() * IMAGE_SIZE() * 3, 0);
	for (size_t iy = 0; iy < IMAGE_SIZE(); ++iy)
	{
		for (size_t ix = 0; ix < IMAGE_SIZE(); ++ix)
		{
			Vec2 p = Vec2{(float(ix) + 0.5f) / float(IMAGE_SIZE()), (float(iy) + 0.5f) / float(IMAGE_SIZE()) };

			float dist = FLT_MAX;
			float triangleShade = 1.0f;
			for (size_t triangleIndex = 0; triangleIndex < g_indices.size(); ++triangleIndex)
			{
				const IVec3& triangle = g_indices[triangleIndex];

				const Vec2& triA = g_vertices[triangle[0]];
				const Vec2& triB = g_vertices[triangle[1]];
				const Vec2& triC = g_vertices[triangle[2]];

				dist = std::min(dist, DistanceFromLineToPoint(triA, triB, p));
				dist = std::min(dist, DistanceFromLineToPoint(triB, triC, p));
				dist = std::min(dist, DistanceFromLineToPoint(triC, triA, p));

				if (PointInTriangle(triA, triB, triC, p))
				{
					triangleShade = float(triangleIndex) / float(g_indices.size() - 1);
					break;
				}
			}

			// TODO: could smoothstep and such
			// TODO: could color the inside of triangles differently
			// TODO: could color triangles based on order, so we can see the order they were added in

			if (dist < 2.0f / float(IMAGE_SIZE()))
			{
				pixels[(iy * IMAGE_SIZE() + ix) * 3 + 0] = 0;
				pixels[(iy * IMAGE_SIZE() + ix) * 3 + 1] = 64;
				pixels[(iy * IMAGE_SIZE() + ix) * 3 + 2] = 0;
			}
			else
			{
				unsigned char shade = (unsigned char)(triangleShade * 255.0f);

				pixels[(iy * IMAGE_SIZE() + ix) * 3 + 0] = shade;
				pixels[(iy * IMAGE_SIZE() + ix) * 3 + 1] = shade;
				pixels[(iy * IMAGE_SIZE() + ix) * 3 + 2] = shade;
			}
		}
	}

	stbi_write_png("out.png", IMAGE_SIZE(), IMAGE_SIZE(), 3, pixels.data(), 0);
}

int main(int argc, char** argv)
{
	MakeMesh();

	return 0;
}

/*
TODO:

NOTE:
- the mesh will never connect with itself. it seems like maybe it should
- maybe we could do better by putting random points on rectangle and breaking it into a triangle mesh with those verts?
*/
