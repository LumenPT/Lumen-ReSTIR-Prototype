#include <cstdio>
#include "RayTracer.h"

#define RANDFLOAT (static_cast<float>(rand()) / static_cast<float>(RAND_MAX))

int main()
{
	RayTracer* tracer = new RayTracer();

	///*
	// * Generate spheres.
	// */
	//for(int i = 0; i < 7; ++i)
	//{
	//	float x = RANDFLOAT * 100.f - 50.f;
	//	float y = RANDFLOAT * 100.f - 50.f;
	//	float z = RANDFLOAT * -100.f - 10.f;

	//	float r = std::fmaxf(0.5f, RANDFLOAT);
	//	float g = std::fmaxf(0.5f, RANDFLOAT);
	//	float b = std::fmaxf(0.5f, RANDFLOAT);

	//	float radius = 10.f;

	//	tracer->AddSphere(Sphere(radius, linalg::aliases::float3{ x, y, z }, { false, linalg::aliases::float3(r, g, b) }));
	//}


	///*
	// * Generate light emitting spheres.
	// */
	//const int NUM_LIGHTS = 10;
	//for (int i = 0; i < NUM_LIGHTS; ++i)
	//{
	//	float x = RANDFLOAT * 40.f - 20.f;
	//	float y = RANDFLOAT * 40.f - 20.f;
	//	float z = RANDFLOAT * -100.f + 50.f;

	//	float r = 1.f;//RANDFLOAT;
	//	float g = 1.f;//RANDFLOAT;
	//	float b = 1.f;//RANDFLOAT;

	//	float radius = RANDFLOAT * 2.f + 0.1f;
	//	float intensity = RANDFLOAT + 1.f;

	//	tracer->AddSphere(Sphere(radius, linalg::aliases::float3{ x, y, z }, { true, intensity * linalg::aliases::float3(r, g, b) }));
	//}
	//


	for (int i = 0; i < 10; ++i)
	{
		float x = (10 * i) - (50.f);
		float y = 0.f;
		float z = -50.f;

		float r = std::fmaxf(0.5f, RANDFLOAT);
		float g = std::fmaxf(0.5f, RANDFLOAT);
		float b = std::fmaxf(0.5f, RANDFLOAT);

		float radius = 10.f;

		tracer->AddSphere(Sphere(radius, linalg::aliases::float3{ x, y, z }, { false, linalg::aliases::float3(r, g, b) }));
	}


	/*
	 * Generate light emitting spheres.
	 */
	const int NUM_LIGHTS = 10;

	float start = -50.f;
	float step = 100.f / static_cast<float>(NUM_LIGHTS);
	float intensityMultiplier = 0.5f;

    for (int i = 0; i < NUM_LIGHTS; ++i)
	{
		float x = start + (step * static_cast<float>(i));
		float y = 0.f;
		float z = 0.f;

		float r = RANDFLOAT + 0.3f;
		float g = RANDFLOAT + 0.3f;
		float b = RANDFLOAT + 0.3f;

		float radius = RANDFLOAT + 0.2f;

		tracer->AddSphere(Sphere(radius, linalg::aliases::float3{ x, y, z }, { true, intensityMultiplier * linalg::aliases::float3(r, g, b) }));
	}

	//Large sphere
	tracer->AddSphere(Sphere(1000, linalg::aliases::float3{ 0.f,0.f, -1050.f }, { false, linalg::aliases::float3(1.f, 1.f, 1.f) }));

	//float intensity = 6.f;
	//tracer->AddSphere(Sphere(1, linalg::aliases::float3{ -30.f, 0.f, -50.f }, { true, intensity * linalg::aliases::float3(1.f, 1.f, 1.f) }));
	//tracer->AddSphere(Sphere(0.2f, linalg::aliases::float3{ -30.f, 10.f, -50.f }, { true, intensity / 2.f * linalg::aliases::float3(1.f, 1.f, 1.f) }));
	//tracer->AddSphere(Sphere(0.2f, linalg::aliases::float3{ -30.f, -10.f, -50.f }, { true, intensity /2.f * linalg::aliases::float3(1.f, 1.f, 1.f) }));


 //   tracer->AddSphere(Sphere(8, linalg::aliases::float3{ 0.f, 0.f, -70.f }, { false, linalg::aliases::float3(1.f, 0.4f, 0.25f) }));
	//tracer->AddSphere(Sphere(20, linalg::aliases::float3{ 40.f,0.f, -90.f }, { false, linalg::aliases::float3(1.f, 0.4f, 0.25f) }));


 //   tracer->AddSphere(Sphere(10, linalg::aliases::float3{ 0.f, 0.f, -50.f }, { true, intensity * linalg::aliases::float3(1.f, 1.f, 1.f) }));
	//tracer->AddSphere(Sphere(10, linalg::aliases::float3{ -30.f, 0.f, -50.f }, { false, linalg::aliases::float3(1.f, 0.4f, 0.25f) }));
	//tracer->AddSphere(Sphere(10, linalg::aliases::float3{ 30.f,0.f, -50.f }, { false, linalg::aliases::float3(1.f, 0.4f, 0.25f) }));
	//tracer->AddSphere(Sphere(10, linalg::aliases::float3{ 0.f,30.f, -50.f }, { false, linalg::aliases::float3(1.f, 0.4f, 0.25f) }));
	//tracer->AddSphere(Sphere(10, linalg::aliases::float3{ 0.f,-30.f, -50.f }, { false, linalg::aliases::float3(1.f, 0.4f, 0.25f) }));

    /*
	 * Create a window and run the algorithms.
	 */
	tracer->SetBackground({ 0.f, 0.1f, 0.3f });
	tracer->Run();
	
	return 0;
}