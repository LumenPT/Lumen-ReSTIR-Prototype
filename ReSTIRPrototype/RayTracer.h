#pragma once
#include "linalg.h"
#include "Data.h"
#include "Restir.h"
#include <SFML/Graphics.hpp>

#include "ThreadPool.h"

class RayTracer
{
public:

	RayTracer() : m_Restir(*this), m_ThreadPool(new ThreadPool(std::thread::hardware_concurrency())) {}

	/*
	 * Generate rays and trace them against the spheres.
	 *
	 */
	void Run();

	void TraceRow(int a_X);

	/*
	 * See if a hit occurs within the bound distance.
	 * An error margin can be specified for the initial ray offset.
	 */
	bool HitWithin(float a_Bound, float a_ErrorMargin, const Ray& a_Ray);

	/*
	 * Generate a ray for the given pixel from the camera position.
	 */
	Ray CalcCameraRay(int x, int y);

	/*
	 * Trace a ray and return hit information.
	 * An error margin can be specified to prevent acne and self shadows.
	 */
	HitInfo TraceRay(float a_ErrorMargin, const Ray& a_Ray) const;

	/*
	 * Add a sphere to the scene.
	 */
	void AddSphere(const Sphere& a_Sphere);

	/*
	 * Set the BG color.
	 */
	void SetBackground(const linalg::aliases::float3& a_Background);

private:
	std::vector<Sphere> m_Spheres;
	ReSTIR m_Restir;
    linalg::aliases::float3 m_Background;
	std::vector<Light> m_Lights;

	//So restir can use it for a bit more speed.
public:
	ThreadPool* m_ThreadPool;
};
