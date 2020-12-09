#include "RayTracer.h"

#include "Data.h"
#include "Restir.h"
#include "ThreadPool.h"

#define GROUND_TRUTH 0
#define DISPLAY_FRAME_COUNT 0

void RayTracer::Run()
{
	//Window for output.
	sf::RenderWindow window(sf::VideoMode(SCREEN_WIDTH, SCREEN_HEIGHT), "ReSTIR!", sf::Style::Default);

	sf::Texture texture;
	texture.create(SCREEN_WIDTH, SCREEN_HEIGHT);
	texture.update(window);
	sf::Sprite sprite(texture);

	sf::Uint8* pixels = new sf::Uint8[SCREEN_WIDTH * SCREEN_HEIGHT * 4];

	while (window.isOpen())
	{
		sf::Event event;
		while (window.pollEvent(event))
		{
			if (event.type == sf::Event::Closed)
			{
				//Close the window
				window.close();
			}
		}

		//How many tasks to perform in the main thread.
		int numTasksInMainThread = SCREEN_WIDTH / m_ThreadPool->numThreads();

		//Generate initial camera rays in other threads.
		for (int x = numTasksInMainThread; x < SCREEN_WIDTH; ++x)
		{
			m_ThreadPool->enqueue([x, this]() {TraceRow(x); });
		}

		//Main thread can't be idle the entire time so do part of the work.
		for(int x = 0; x < numTasksInMainThread; ++x)
		{
			TraceRow(x);
		}

		//Once the main thread is done, wait for worker threads to also be done.
		bool wait = true;
		while(wait)
		{
			wait = m_ThreadPool->numBusyThreads() != 0;
		}

		//Run restir on generated rays.
		m_Restir.Run(m_Lights);

		//Perform shading per pixel based on ReSTIR reservoirs.
		for (int y = 0; y < SCREEN_HEIGHT; y++)
		{
			for (int x = 0; x < SCREEN_WIDTH; x++)
			{
				const auto& pixel = m_Restir.GetPixelData(x, y);

				//The final pixel color.
				linalg::aliases::float3 color;

				//No hit so just BG color.
				if (pixel.depth < 0.005f)
				{
					color = m_Background;
				}
				//Emissive is just the emissive color.
				else if (pixel.material.emissive)
				{
					color = pixel.material.color;
				}
				else
				{
#if GROUND_TRUTH

					int numIterations = 1000;
					for (int itr = 0; itr < numIterations; ++itr)
					{


						//Average the reservoir results.
						for (int i = 0; i < m_Lights.size(); ++i)
						{
							const float x = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
							const float y = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
							auto sample = m_Lights[i].CalculateSample(linalg::aliases::float2(x, y), pixel);
							auto toLightVec = sample.position - pixel.position;
							float lDist = linalg::length(toLightVec);
							toLightVec /= lDist;
							const float lambertian = linalg::dot(toLightVec, pixel.normal);
							auto color1 = sample.radiance * pixel.material.color;

							Ray r = { sample.position, -toLightVec };

							//Apply shading if not occluded.
							if (!HitWithin(lDist, 0.005f, r))
							{
								color += (color1 * lambertian) / 3.14159265f;
							}
				        }
			        }

					//Take the average.
					color /= static_cast<float>(numIterations);

					const double total = SCREEN_WIDTH * SCREEN_HEIGHT;
					const double current = SCREEN_WIDTH * y + x;

					double pct = current / total * 100.f;
					std::cout << "Ground truth progress: " << pct << "%" << std::endl;

#else
					//Average the reservoir results.
					for (int i = 0; i < NUM_RESERVOIRS_PER_PIXEL; ++i)
					{
						auto& reservoir = pixel.lightReservoirs[i];
						auto toLightVec = linalg::normalize(reservoir.sample.position - pixel.position);
						const float lambertian = linalg::dot(toLightVec, pixel.normal);
						auto color1 = reservoir.sample.radiance * pixel.material.color;

						//Apparently dividing by pi makes this physically correct (I guess because pi is half a hemisphere).
						color += ((static_cast<float>(reservoir.weight) * color1 * lambertian) / 3.14159265f);
					}

					color /= static_cast<float>(NUM_RESERVOIRS_PER_PIXEL);
#endif
				}

				//Clamp to the 0-1 range.
				color.x = std::clamp(color.x, 0.f, 1.f); color.x = std::clamp(color.x, 0.f, 1.f);
				color.y = std::clamp(color.y, 0.f, 1.f); color.x = std::clamp(color.x, 0.f, 1.f);
				color.z = std::clamp(color.z, 0.f, 1.f); color.x = std::clamp(color.x, 0.f, 1.f);

				//Enable to test normals of surfaces.
				//color = linalg::aliases::float3((pixel.normal.x + 1.f) / 2.f,(pixel.normal.y + 1.f) / 2.f,	(pixel.normal.z + 1.f) / 2.f);

				const int index = (y * SCREEN_WIDTH * 4) + (x * 4);

				pixels[index] = static_cast<std::uint8_t>(color.x * 255);
				pixels[index + 1] = static_cast<std::uint8_t>(color.y * 255);
				pixels[index + 2] = static_cast<std::uint8_t>(color.z * 255);
				pixels[index + 3] = 255;
			}
		}

		//Write pixels to the texture.
		texture.update(pixels);

		//Display on the screen.
		window.clear();
		window.draw(sprite);
		window.display();

		//Swap the ReSTIR buffers. The current frames buffer will now be used for temporal samples.
		m_Restir.SwapBuffers();

#if DISPLAY_FRAME_COUNT
		static int frame = 0;
		std::cout << "Frame " << frame << std::endl;
		++frame;
#endif
	}
}

void RayTracer::TraceRow(int a_X)
{
	for (int y = 0; y < SCREEN_HEIGHT; ++y)
	{
		const auto ray = CalcCameraRay(a_X, y);
		const auto info = TraceRay(0.05f, ray);

		//Hit!
		if (info.distance > 0.f)
		{
			m_Restir.SetPixelData(a_X, y, (info.distance * ray.direction) + ray.origin, info.normal, info.distance, info.material);
		}
		//No hit
		else
		{
			m_Restir.SetPixelData(a_X, y, linalg::aliases::float3(0.f), linalg::aliases::float3(0.f), -1.f, Material());
		}
	}
}

bool RayTracer::HitWithin(float a_Bound, float a_ErrorMargin, const Ray& a_Ray)
{
	HitInfo current;
	for(auto& sphere : m_Spheres)
	{
		current = sphere.Intersects(a_Ray);

		if(current.distance > a_ErrorMargin && current.distance <= a_Bound - a_ErrorMargin)
		{
			return true;
		}
	}
	return false;
}

Ray RayTracer::CalcCameraRay(int x, int y)
{
	//The Z coordinate is -1 because the image projection is one unit away from the camera to make the FOV calculation possible.
	//Pointing into negative Z to follow OpenGL and other implementations.
	float pX, pY, pZ = -1;

	//Normalize to the range 0 - 1
	const float normalizedX = (static_cast<float>(x) + 0.5F) / SCREEN_WIDTH;
	const float normalizedY = (static_cast<float>(y) + 0.5F) / SCREEN_HEIGHT;

	//Aspect ratio is the ratio of X to Y pixels.
	//const float aspectRatio = static_cast<float>(width) / static_cast<float>(height);
	//const float fovModifier = tan((fov * 0.5f) * (3.141592f / 180.f));

	float fov = 70.f;
    float fovModifier = tan((fov * 0.5f) * (3.141592f / 180.f));
	float aspectRatio = SCREEN_WIDTH / SCREEN_HEIGHT;
	float fovAspect = fovModifier * aspectRatio;

	//pX = (2.f * normalizedX - 1.f) * fovModifier * aspectRatio;
	//pY = (1.f - 2.f * normalizedY) * fovModifier;

	pX = (2.f * normalizedX - 1.f) * fovAspect;
	pY = (1.f - 2.f * normalizedY) * fovModifier;

    const linalg::aliases::float3 direction = linalg::normalize(linalg::aliases::float3{ pX, pY, pZ });

	const linalg::aliases::float3 origin = {0.f, 0.f, 0.f};

	//Create a ray with the translated direction.
	//The origin of the ray is the {0, 0, 0} origin multiplied by the transformation (rotates and translates).
	//Direction is automatically normalized in the ray constructor.
	return { origin, direction };
}

HitInfo RayTracer::TraceRay(float a_ErrorMargin, const Ray& a_Ray) const
{
	HitInfo closestHit;
	closestHit.distance = std::numeric_limits<float>::max();
	bool hit = false;
	HitInfo current;
	for (auto& sphere : m_Spheres)
	{
		current = sphere.Intersects(a_Ray);

		if (current.distance > a_ErrorMargin && current.distance < closestHit.distance)
		{
			closestHit = current;
			hit = true;
		}
	}

	if(!hit)
	{
		closestHit.distance = -1.f;
	}

	return closestHit;
}

void RayTracer::AddSphere(const Sphere& a_Sphere)
{
	if(a_Sphere.GetMaterial().emissive)
	{
		PackedLightData sphereData;
		sphereData.position = a_Sphere.GetCenter();
		sphereData.radiance = a_Sphere.GetMaterial().color;
		sphereData.radius = a_Sphere.GetRadius();
		sphereData.type = LightType::SPHERE_LIGHT;
		m_Lights.emplace_back(Light(sphereData));
	}
	//Dont display lights for now for performance on CPU.
    else
	{
		m_Spheres.push_back(a_Sphere);
	}
}

void RayTracer::SetBackground(const linalg::aliases::float3& a_Background)
{
	m_Background = a_Background;
}
