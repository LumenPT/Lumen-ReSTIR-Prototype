#include "Restir.h"
#include <algorithm>
#include <iostream>

#include "RayTracer.h"

void ReSTIR::Run(const std::vector<Light>& a_Lights)
{
	assert(!a_Lights.empty() && "Cannot run restir without lights.");

	//For now ensure resolution is a multiple of the grid size.
	assert(SCREEN_WIDTH % PIXEL_GRID_SIZE == 0 && SCREEN_HEIGHT % PIXEL_GRID_SIZE == 0);

	/*
	 * Step 1. Generate a CDF (Cumulative distribution function) of all lights in the scene based on their irradiance.
	 */

    //TODO  This does not prioritize certain colors, but humans see green better. Maybe weigh based on human perception?

    //CDF containing every light based on weight. Lights can be picked with a normalized random float.
	CDF<const Light*> cdf(a_Lights.size());
	for(int i = 0; i < static_cast<int>(a_Lights.size()); ++i)
	{
		const Light& light = a_Lights[i];
		auto vec = light.data.radiance;
		cdf.Append(&light, (vec.x + vec.y + vec.z) / 3.f);
	}

	/*
	 * Step 2. Generate light bags by picking lights from the CDF.
	 * I don't think it makes sense to generate more bags than there are pixel grids.
	 * This is why the pixel grid is calculated here.
	 */
	const int numXGrids = (SCREEN_WIDTH / PIXEL_GRID_SIZE);
	const int numYGrids = (SCREEN_HEIGHT / PIXEL_GRID_SIZE);
	const int numPixelGrids = numXGrids * numYGrids;

	//Generate one light bag per pixel grid.
	std::vector<std::vector<std::pair<Light, float>>> lightBags;
	lightBags.reserve(numPixelGrids);

	for(int i = 0; i < numPixelGrids; ++i)
	{
		lightBags.emplace_back(std::vector<std::pair<Light, float>>());
		lightBags[i].reserve(NUM_LIGHTS_PER_BAG);

		const Light* lp = nullptr;
		float weight = 0;

		for(int j = 0; j < NUM_LIGHTS_PER_BAG; ++j)
		{
			//Find a random weighed light in the CDF and store its PDF (the first crude one).
			cdf.FindRandom(lp, weight);
			lightBags[i].emplace_back(*lp, weight);
		}
	}

	/*
	 * Step 3. Iterate over every pixel in the image (16x16 region at a time). One light bag is used per region. Choose a number of random samples per bag.
	 * For each picked light create a LightSample and use the SolidAnglePDF as weight for the reservoirs.
	 * Do this for every reservoir for every pixel.
	 */
	for(int gridX = 0; gridX < numXGrids; ++gridX)
	{
	    for(int gridY = 0; gridY < numYGrids; ++gridY)
	    {
			const int lightBagIndex = (numXGrids * gridY) + gridX;

	        for(int x = 0; x < PIXEL_GRID_SIZE; ++x)
	        {
	            for(int y = 0; y < PIXEL_GRID_SIZE; ++y)
	            {
					//X,Y position within this grid cell.
					const int posX = PIXEL_GRID_SIZE * gridX + x;
					const int posY = PIXEL_GRID_SIZE * gridY + y;

					//The pixel being updated.
					int bufferIndex = static_cast<int>(m_SwapIndex);
					auto& pixel = m_Images[bufferIndex].At(posX, posY);

					//Only run if the pixel has a hit.
					if (pixel.depth > 0.f)
					{
						//Sample iteration (multiple reservoirs per pixel).
						for (int depth = 0; depth < NUM_RESERVOIRS_PER_PIXEL; ++depth)
						{
							auto& reservoir = pixel.lightReservoirs[depth];

							//Reset reservoir so that a clean slate is had.
							reservoir.Reset();

							//Generate the amount of samples specified per reservoir.
							for (int sample = 0; sample < NUM_PRELIMINARY_SAMPLES; ++sample)
							{
								//Random index in light bag.
								const auto index = static_cast<int>((static_cast<float>(rand()) / static_cast<float>(RAND_MAX)) * static_cast<float>((NUM_LIGHTS_PER_BAG - 1)));
								auto& pair = lightBags[lightBagIndex][index];
								auto& light = pair.first;

								//Generate random UV coordinates. Between 0 and 1.
								const float u = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
								const float v = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
								
								//Generate a sample with solid angle PDF for this specific pixel.
								const auto lightSample = light.CalculateSample(linalg::aliases::float2(u, v), pixel);

								//The final PDF for the light in this reservoir is the solid angle divided by the original PDF of the light being chosen based on radiance.
								const auto pdf = lightSample.solidAnglePdf / pair.second;
								reservoir.Update(lightSample, pdf);
							}

							//Calculate the weight of the reservoir based on the current values inside it.
							reservoir.UpdateWeight();
						}
					}
	            }
	        }
	    }
	}

	/*
	 * Step 4. Check the visibility of the remaining sample. Set weight to 0 if not visible and discard.
	 * This step can be optimized by batching up all shadow rays and then processing them afterward.
	 */
	//Shoot shadow rays to test visibility.

	for (int depth = 0; depth < NUM_RESERVOIRS_PER_PIXEL; ++depth)
	{
		m_RayTracer.m_ThreadPool->enqueue([depth, this]() mutable
		{
			for (int y = 0; y < SCREEN_HEIGHT; ++y)
			{
				for (int x = 0; x < SCREEN_WIDTH; ++x)
				{
					int bufferIndex = static_cast<int>(m_SwapIndex);
					auto& pixel = m_Images[bufferIndex].At(x, y);

					if (pixel.depth > 0.f)
					{
						//Cast shadow rays and set weight to 0 if occluded.
						auto& reservoir = pixel.lightReservoirs[depth];
						linalg::aliases::float3 lDir = pixel.position - reservoir.sample.position;
						float lDist = linalg::length(lDir);
						lDir /= lDist;

						Ray r = { reservoir.sample.position, lDir };

						//If not hit in distance, set weight to 0.
						if (m_RayTracer.HitWithin(lDist, 0.005f, r))
						{
							reservoir.weight = 0.f;
						}
					}
				}
			}
		});
	}


	//Wait for the shadow rays to be traced.
	bool running = true;
	while(running)
	{
	    if(m_RayTracer.m_ThreadPool->numBusyThreads() == 0)
	    {
			running = false;
	    }
	}

#if TEMPORAL_SAMPLING
	/*
	 * Step 5. Look at temporal samples and compare with the reservoir. This uses last frames reservoirs.
	 * A motion vector is required if the scene is dynamic to be able to look up corresponding pixels.
	 * Discard samples that are too different.
	 * Since temporal reservoirs can grow unbounded in size, clamp them at 20 samples.
	 * TODO: Motion vectors. For now pick same pixel.
	 */
	for (int x = 0; x < SCREEN_WIDTH; ++x)
	{
	    for (int y = 0; y < SCREEN_HEIGHT; ++y)
	    {
			int bufferIndex = static_cast<int>(m_SwapIndex);
			int otherBufferIndex = static_cast<int>(!m_SwapIndex);

			auto& currentPixel = m_Images[bufferIndex].At(x, y);
			auto& temporalPixel = m_Images[otherBufferIndex].At(x, y);

			if (temporalPixel.depth > 0.f && currentPixel.depth > 0.f)
			{
				for (int depth = 0; depth < NUM_RESERVOIRS_PER_PIXEL; ++depth)
				{
					//TODO motion vectors.

#if BIASED 
					std::array<Reservoir*, 2> toCombine;
					toCombine[0] = &temporalPixel.lightReservoirs[depth];
					toCombine[1] = &currentPixel.lightReservoirs[depth];

					//Cap sample count at 20x current to reduce temporal influence. Would grow infinitely large otherwise.
					toCombine[0]->sampleCount = std::min(toCombine[0]->sampleCount, toCombine[1]->sampleCount * 20);

					CombineBiased(currentPixel, toCombine, 2, depth);

#else
					std::array<Pixel*, 2> toCombine;
					toCombine[0] = &temporalPixel;
					toCombine[1] = &currentPixel;

					//Cap sample count at 20x current to reduce temporal influence. Would grow infinitely large otherwise.
					//TODO Maybe this capping invalidates the weight of the temporal reservoir? I don't think it does but it's a possibility. If results are wrong look here.
					toCombine[0]->lightReservoirs[depth].sampleCount = std::min(toCombine[0]->lightReservoirs[depth].sampleCount, toCombine[1]->lightReservoirs[depth].sampleCount * 20);

					CombineUnbiased(currentPixel, toCombine, 2, depth);
#endif
				}
			}
		}
	}

#endif

#if SPATIAL_SAMPLING
	/*
	 * Step 6. Look at a certain amount of spatial samples in a radius around each pixel.
	 * Discard neighbours at high depth and normal difference. 25 degree angle difference and 10% depth difference are discarded.
	 */

	//Source and output buffers to store spatial sample combinations.
	int from = static_cast<int>(m_SwapIndex);
	int to = 2;

	//Copy the pixel data so that this algorithm will have the right pixel depth information even in the other buffer. far from optimal but it's a prototype.
	m_Images[to] = m_Images[from];

	for(int iteration = 0; iteration < SPATIAL_ITERATIONS; ++iteration)
	{
		for (int y = 0; y < SCREEN_HEIGHT; ++y)
		{
			for (int x = 0; x < SCREEN_WIDTH; ++x)
			{
				//Sample from the input buffer.
				auto& currentPixel = m_Images[from].At(x, y);

				if (currentPixel.depth > 0.f)
				{
					for (int depth = 0; depth < NUM_RESERVOIRS_PER_PIXEL; ++depth)
					{
#if BIASED
						std::array<Reservoir*, NUM_NEIGHBOUR_SAMPLES + 1> toCombineSpatial;
						toCombineSpatial[0] = &currentPixel.lightReservoirs[depth];

						int count = 1;

						for (int i = 1; i <= NUM_NEIGHBOUR_SAMPLES; ++i)
						{
							const float neighbourY = (((static_cast<float>(rand()) / static_cast<float>(RAND_MAX)) * 2.f) - 1.f) * static_cast<float>(NEIGHBOUR_COMPARISON_RADIUS);
							const float neighbourX = (((static_cast<float>(rand()) / static_cast<float>(RAND_MAX)) * 2.f) - 1.f) * static_cast<float>(NEIGHBOUR_COMPARISON_RADIUS);

							//Clamp the values in case they are outside of the image.
							int pickedX = std::ceil(neighbourX + static_cast<float>(x));
							int pickedY = std::ceil(neighbourY + static_cast<float>(y));
							pickedX = std::clamp(pickedX, 0, SCREEN_WIDTH - 1);
							pickedY = std::clamp(pickedY, 0, SCREEN_HEIGHT - 1);


							auto& pickedPixel = m_Images[from].At(pickedX, pickedY);

							//Discard samples that are too different.
							float depth1 = pickedPixel.depth;
							float depth2 = currentPixel.depth;
							float depthDifPct = fabs(depth1 - depth2) / ((depth1 + depth2) / 2.f);

							float angleDif = linalg::dot(pickedPixel.normal, currentPixel.normal);	//Between 0 and 1 (0 to 90 degrees). 
							static constexpr float MAX_ANGLE_COS = 0.72222222223f;	//Dot product is cos of the angle. If higher than this value, it's within 25 degrees.

							if (pickedPixel.depth > 0.f && depthDifPct < 0.10f && angleDif > MAX_ANGLE_COS)
							{
								assert(pickedPixel.lightReservoirs[depth].weight >= 0.f);
								toCombineSpatial[count] = &pickedPixel.lightReservoirs[depth];
								++count;
							}
						}

						if (count > 1)
						{
							//Output to the output buffers pixel at the same position. 
							CombineBiased(m_Images[to].At(x, y), toCombineSpatial, count, depth);
						}
#else
						std::array<Pixel*, NUM_NEIGHBOUR_SAMPLES + 1> toCombineSpatial;
						toCombineSpatial[0] = &currentPixel;

						int count = 1;
						for (int i = 1; i <= NUM_NEIGHBOUR_SAMPLES; ++i)
						{
							//Generate a random point in the radius.
							const float neighbourY = (((static_cast<float>(rand()) / static_cast<float>(RAND_MAX)) * 2.f) - 1.f) * static_cast<float>(NEIGHBOUR_COMPARISON_RADIUS);
							const float neighbourX = (((static_cast<float>(rand()) / static_cast<float>(RAND_MAX)) * 2.f) - 1.f) * static_cast<float>(NEIGHBOUR_COMPARISON_RADIUS);

							//Clamp the values in case they are outside of the image.
							int pickedX = std::ceil(neighbourX + static_cast<float>(x));
							int pickedY = std::ceil(neighbourY + static_cast<float>(y));
							pickedX = std::clamp(pickedX, 0, SCREEN_WIDTH - 1);
							pickedY = std::clamp(pickedY, 0, SCREEN_HEIGHT - 1);

							auto& pickedPixel = m_Images[from].At(pickedX, pickedY);

							//Discard samples that are too different.
							float depth1 = pickedPixel.depth;
							float depth2 = currentPixel.depth;
							float depthDifPct = abs(depth1 - depth2) / ((depth1 + depth2) / 2.f);

							float angleDif = linalg::dot(pickedPixel.normal, currentPixel.normal);	//Between 0 and 1 (0 to 90 degrees). 
							static constexpr float MAX_ANGLE_COS = 0.72222222223f;	//Dot product is cos of the angle. If higher than this value, it's within 25 degrees.

							if (pickedPixel.depth > 0.f && depthDifPct < 0.10f && angleDif > MAX_ANGLE_COS)
							{
								toCombineSpatial[count] = &pickedPixel;
								++count;
							}
						}

						if (count > 1)
						{
							//Output to the output buffers pixel at the same position.
							CombineUnbiased(m_Images[to].At(x, y), toCombineSpatial, count, depth);
						}
#endif
					}
				}
			}
		}

		//Swap the input and output buffers. 
		std::swap(from, to);
	}

	//Copy results back to the current buffer if the amount of iterations did not end up there already.
	if(SPATIAL_ITERATIONS % 2 != 0)
	{
	    //from and to were just swapped, so this seems wrong but it's correct.
		m_Images[to] = m_Images[from];
	}

#endif

#if TEMPORAL_SAMPLING || SPATIAL_SAMPLING
	/*
	 * Step 7. If the sample chosen is not the original sample, send another shadow ray to check for visibility.
	 */
	for (int depth = 0; depth < NUM_RESERVOIRS_PER_PIXEL; ++depth)
	{
		m_RayTracer.m_ThreadPool->enqueue([depth, this]() mutable
		{
			for (int x = 0; x < SCREEN_WIDTH; ++x)
			{
				for (int y = 0; y < SCREEN_HEIGHT; ++y)
				{
					const int bufferIndex = static_cast<int>(m_SwapIndex);
					auto& pixel = m_Images[bufferIndex].At(x, y);

					if (pixel.depth > 0.f)
					{
						//Cast shadow rays and set weight to 0 if occluded.
						auto& reservoir = pixel.lightReservoirs[depth];
						linalg::aliases::float3 lDir = pixel.position - reservoir.sample.position;
						const float lDist = linalg::length(lDir);
						lDir /= lDist;

						Ray r = { reservoir.sample.position, lDir };

						//If not hit in distance, set weight to 0.
						if (m_RayTracer.HitWithin(lDist, 0.005f, r))
						{
							reservoir.weight = 0.f;
						}
					}
				}
			}
		});
	}


	//Wait for the shadow rays to be traced.
	running = true;
	while (running)
	{
		if (m_RayTracer.m_ThreadPool->numBusyThreads() == 0)
		{
			running = false;
		}
	}
#endif
}

void ReSTIR::SwapBuffers()
{
	/*
     * Step 8. Swap around the reservoir buffers.
     * This happens first so that after running restir, the buffers are not yet swapped so other processes can read the data.
     * Otherwise they have no way of knowing which buffer to look in.
     */
	m_SwapIndex = !m_SwapIndex;
}

void ReSTIR::SetPixelData(int a_X, int a_Y, const linalg::aliases::float3& a_Position,
                          const linalg::aliases::float3& a_Normal, float a_Depth, const Material& a_Material)
{
	int bufferIndex = static_cast<int>(m_SwapIndex);
	auto& pixel = m_Images[bufferIndex].At(a_X, a_Y);
	pixel.position = a_Position;
	pixel.depth = a_Depth;
	pixel.normal = a_Normal;
	pixel.material = a_Material;
	pixel.pixelCoords = linalg::aliases::float2(a_X, a_Y);
}
