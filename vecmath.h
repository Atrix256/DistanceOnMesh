#pragma once

#include <array>

template <size_t N>
using Vec = std::array<float, N>;

template <size_t N>
using IVec = std::array<int, N>;

using Vec2 = Vec<2>;
using Vec3 = Vec<3>;
using IVec2 = IVec<2>;
using IVec3 = IVec<3>;

template <typename T, size_t N>
inline std::array<T, N> operator - (const std::array<T, N>& A, const std::array<T, N>& B)
{
	std::array<T, N> ret;
	for (size_t i = 0; i < N; ++i)
		ret[i] = A[i] - B[i];
	return ret;
}

template <typename T, size_t N>
inline std::array<T, N> operator + (const std::array<T, N>& A, const std::array<T, N>& B)
{
	std::array<T, N> ret;
	for (size_t i = 0; i < N; ++i)
		ret[i] = A[i] + B[i];
	return ret;
}

template <typename T, size_t N>
inline std::array<T, N> operator * (const std::array<T, N>& A, float B)
{
	std::array<T, N> ret;
	for (size_t i = 0; i < N; ++i)
		ret[i] = A[i] * B;
	return ret;
}

template <typename T, size_t N>
inline std::array<T, N> operator / (const std::array<T, N>& A, float B)
{
	std::array<T, N> ret;
	for (size_t i = 0; i < N; ++i)
		ret[i] = A[i] / B;
	return ret;
}

template <typename T, size_t N>
inline T Dot(const std::array<T, N>& A, const std::array<T, N>& B)
{
	T ret = T(0);
	for (size_t i = 0; i < N; ++i)
		ret += A[i] * B[i];
	return ret;
}

template <size_t N>
inline float Length(const Vec<N>& V)
{
	float length = 0.0f;
	for (float f : V)
		length += f * f;
	return std::sqrt(length);
}

template <size_t N>
inline float Distance(const Vec<N>& A, const Vec<N>& B)
{
	return Length(B - A);
}

template <size_t N>
inline Vec<N> Normalize(const Vec<N>& V)
{
	float length = Length(V);

	Vec<N> ret;
	for (size_t i = 0; i < N; ++i)
		ret[i] = V[i] / length;
	return ret;
}

// from "Real time collision detection"
inline float Signed2DTriArea(const Vec2& A, const Vec2& B, const Vec2& C)
{
	return (A[0] - C[0]) * (B[1] - C[1]) - (A[1] - C[1]) * (B[0] - C[0]);
}

// from "Real time collision detection"
inline Vec3 Barycentric(const Vec2& A, const Vec2& B, const Vec2& C, const Vec2& P)
{
	Vec2 v0 = B - A;
	Vec2 v1 = C - A;
	Vec2 v2 = P - A;
	float d00 = Dot(v0, v0);
	float d01 = Dot(v0, v1);
	float d11 = Dot(v1, v1);
	float d20 = Dot(v2, v0);
	float d21 = Dot(v2, v1);
	float denom = d00 * d11 - d01 * d01;

	Vec3 ret;
	ret[1] = (d11 * d20 - d01 * d21) / denom;
	ret[2] = (d00 * d21 - d01 * d20) / denom;
	ret[0] = 1.0f - ret[1] - ret[2];
	return ret;
}

inline float DistanceFromLineToPoint(const Vec2& A, const Vec2& B, const Vec2& P)
{
	Vec2 AB = B - A;
	float ABLen = Length(AB);
	AB = Normalize(AB);
	Vec2 AP = P - A;

	float t = Dot(AP, AB);
	
	Vec2 closestPoint;
	if (t <= 0.0f)
		closestPoint = A;
	else if (t >= ABLen)
		closestPoint = B;
	else
		closestPoint = A + AB * t;

	return Distance(closestPoint, P);
}

// from "Real time collision detection"
inline bool LinesIntersect(const Vec2& A, const Vec2& B, const Vec2& C, const Vec2& D)
{
	float a1 = Signed2DTriArea(A, B, D);
	float a2 = Signed2DTriArea(A, B, C);

	if (a1 * a2 < 0.0f)
	{
		float a3 = Signed2DTriArea(C, D, A);
		float a4 = a3 + a2 - a1;
		if (a3 * a4 < 0.0f)
		{
			// t = a3 / (a3 - a4)
			// p = a + t * (b-a)
			return true;
		}
	}

	return false;
}

inline bool PointInTriangle(const Vec2& A, const Vec2& B, const Vec2& C, const Vec2& P)
{
	Vec3 bary = Barycentric(A, B, C, P);
	return (bary[0] >= 0.0f && bary[0] <= 1.0f &&
			bary[1] >= 0.0f && bary[1] <= 1.0f &&
			bary[2] >= 0.0f && bary[2] <= 1.0f);
}