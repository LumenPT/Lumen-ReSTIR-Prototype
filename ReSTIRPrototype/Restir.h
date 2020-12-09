#pragma once

#include <cassert>
#include <iostream>
#include <vector>

#include "Data.h"
#include "linalg.h"

/*
 * DEFINES
 */

//If 1, biased algorithm is used. 0 disables bias but increases runtime.
#define BIASED 0

//Screen dimensions
#define SCREEN_WIDTH 336
#define SCREEN_HEIGHT 336

//Amount of reservoirs per pixel. ReSTIR uses 1 or 4depending on biased or unbiased.
#define NUM_RESERVOIRS_PER_PIXEL 5

//The amount of lights in a single bag. Should fit in the GPU L1 Cache. ReSTIR uses 1000 per bag.
#define NUM_LIGHTS_PER_BAG 1000

//The amount of initial samples to take per light bag. Should be smaller than the lights in a bag, 32 is a good amount.
#define NUM_PRELIMINARY_SAMPLES 32 

//How many neighbouring pixels to sample for spatial comparing. ReSTIR uses between 3 and 5 depending on bias or unbias.
#define NUM_NEIGHBOUR_SAMPLES 5

//The pixel radius to look for neighbours in. ReSTIR uses 30.
#define NEIGHBOUR_COMPARISON_RADIUS 30

//The width and height of a pixel grid (uses one light bag per grid). ReSTIR uses 16.
#define PIXEL_GRID_SIZE 16

//Restir uses 2 (or 1 for unbiased).
#define SPATIAL_ITERATIONS 2

//ENABLE OR DISABLE TEMPORAL AND SPATIAL SAMPLING
#define TEMPORAL_SAMPLING 1
#define SPATIAL_SAMPLING 1


struct Material;
//Forward declare.
class RayTracer;

/*
 * LightSample contains information about a light, specific to a surface in the scene.
 * SolidAnglePDF describes the relative probability that this sample affects the surface in question.
 * The light is projected onto the hemisphere 
 */
struct LightSample
{
    LightSample() : radiance(0.f), normal(0.f), position(0.f), solidAnglePdf(0.f) {}

    linalg::aliases::float3 radiance;
    linalg::aliases::float3 normal;
    linalg::aliases::float3 position;
    float solidAnglePdf;
};

enum class LightType
{
    SPHERE_LIGHT
};

struct PackedLightData
{
    linalg::aliases::float3 radiance;
    linalg::aliases::float3 normal;
    linalg::aliases::float3 position;
    float radius;
    LightType type;
};

/*
 * Reservoirs contain a weight and chosen sample.
 * A sample can be any object that has a weight.
 */
struct Reservoir
{
    Reservoir() : weightSum(0.f), sampleCount(0), weight(0.f)
    {

    }

    /*
     * Update this reservoir with a light and a weight for that light relative to the total set.
     */
    bool Update(const LightSample& a_Sample, float a_Weight)
    {
        assert(a_Weight >= 0.f);

        //Append weight to total.
        weightSum += a_Weight;
        ++sampleCount;

        //Generate a random float between 0 and 1 and then overwrite the sample based on the probability of this sample being chosen.
        const float r = static_cast<float>(rand()) / static_cast <float> (RAND_MAX);

        //In this case R is inclusive with 0.0 and 1.0. This means that the first sample is always chosen.
        //If weight is 0, then a division by 0 would happen. Also it'd be impossible to pick this sample.
        if (a_Weight != 0.f && r <= (a_Weight / weightSum))
        {
            sample = a_Sample;
            return true;
        }

        return false;
    }

    /*
     * Calculate the weight for this reservoir.
     */
    void UpdateWeight()
    {
        //If no samples have been considered yet, then the weight is also 0 (prevents division by 0).
        //Also 0 if none of the considered samples contributed.
        if(sampleCount == 0 || weightSum <= 0.f)
        {
            weight = 0;
            return;
        }

        //Can't divide by 0 so take a super tiny PDF instead.
        weight = (1.f / std::max(sample.solidAnglePdf, std::numeric_limits<float>::min())) * ((1.0 / static_cast<float>(sampleCount)) * weightSum);
        assert(weight >= 0.f);
        assert(weight < INFINITY);
    }

    void Reset()
    {
        weightSum = 0.f;
        sampleCount = 0;
        weight = 0.f;
    }

    //The sum of individual samples weight.
    float weightSum;

    //The amount of samples seen.
    std::uint64_t sampleCount;

    //The actual resulting weight. Calculate using UpdateWeight function.
    float weight;
    LightSample sample;
};

/*
 * Information about a pixel.
 */
struct Pixel
{
    Pixel() : position(0.f), normal(0.f), depth(0.f) {}

    linalg::aliases::float2 pixelCoords;
    linalg::aliases::float3 position;
    linalg::aliases::float3 normal;
    float depth;
    Material material;
    Reservoir lightReservoirs[NUM_RESERVOIRS_PER_PIXEL];
};

//Light sampling functions
static auto sampleSphereLight = [](const PackedLightData& a_Data, linalg::aliases::float2 a_Uv, const Pixel& a_Pixel)
{
    LightSample sample;
    sample.radiance = a_Data.radiance;

    //Generate random z and prevent nul vector because can't normalize that one.
    float z = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    if (a_Uv.x == 0 && a_Uv.y == 0 && z == 0)
    {
        z = 1.f;
    }

    //Calculate the solid angle for this light on the given surface.
    linalg::aliases::float3 toLightDir = a_Data.position - a_Pixel.position;                                    //Direction from pixel to light.
    float lDistance = linalg::length(toLightDir);

    //Can't divide by 0.
    if (lDistance <= 0.f)
    {
        sample.solidAnglePdf = 0.f;
        return sample;
    }

    //Light distance from pixel.
    toLightDir /= lDistance;

    //Generate the point on the sphere and normalize it.
    linalg::aliases::float3 pointOnSphere(a_Uv.x * 2.f - 1.f, a_Uv.y * 2.f - 1.f, z * 2.f - 1.f);
    pointOnSphere = linalg::normalize(pointOnSphere);

    //If point on sphere generated is pointing at the wrong hemisphere, simply invert it.
    if (linalg::dot(pointOnSphere, -toLightDir) < 0.f)
    {
        pointOnSphere *= -1.f;
    }

    //Make point actually lie on the sphere
    pointOnSphere *= a_Data.radius;
    pointOnSphere += a_Data.position;

    sample.normal = linalg::normalize(pointOnSphere - a_Data.position);
    sample.position = pointOnSphere;

    //Correct light distance by subtracting radius. Also do this for direction.
    toLightDir = (sample.position - a_Pixel.position);
    lDistance = linalg::length(toLightDir);

    //Can't divide by 0.
    if(lDistance <= 0.f)
    {
        sample.solidAnglePdf = 0.f;
        return sample;
    }

    toLightDir /= lDistance; //This is required in case the sphere radius is larger than lDistance. In that case the surface is inside the light and thus the PDF should be 0.


    //Calculate the solid angle PDF.
    const float dot1 = std::clamp(linalg::dot(toLightDir, a_Pixel.normal), 0.f, 1.f);           //Lambertian term clamped between 0 and 1. SurfaceN dot ToLight
    const float dot2 = std::clamp(linalg::dot(-toLightDir, sample.normal), 0.f, 1.f);

    //Geometry term G(x).
    const float solidAngle = (dot1 * dot2) / (lDistance * lDistance);  //No need to multiply with dot2 here (light normal dot light dir) because those are always equal on a sphere light.
                                                                //UNLESS I add UV coords.

    //BSDF is equal to material color for now.
    const auto& bsdf = a_Pixel.material.color;

    //Light emittance of the light is equal to it's radiance
    const auto& emittance = a_Data.radiance;

    //For the light strenghth/bsdf factors, I take the average of each channel to weigh them.
    const linalg::aliases::float3 colorFactor = bsdf * emittance;
    const float averageColor = (colorFactor.x + colorFactor.y + colorFactor.z) / 3.f;

    sample.solidAnglePdf = (averageColor * solidAngle);

    assert(sample.solidAnglePdf >= 0.f);

    return sample;
};

struct Light
{
    Light(PackedLightData& a_Data)
    {
        data = a_Data;

        switch (data.type)
        {
        case LightType::SPHERE_LIGHT:
            sampleFunction = sampleSphereLight;
            break;
        default:
            throw std::exception("Reeeeeeee");
        }
    }

    /*
     * Take a sample to be stored in a reservoir for a specific pixel and light.
     */
    LightSample CalculateSample(linalg::aliases::float2 a_Uv, const Pixel& a_Pixel) const
    {
        return sampleFunction(data, a_Uv, a_Pixel);
    }

    //Pointer to the sample function used by this light type.
    std::function<LightSample(const PackedLightData& a_Data, linalg::aliases::float2 a_Uv, const Pixel& a_Pixel)> sampleFunction;

    //Light data.
    PackedLightData data;
};


/*
 * SampleBuffer containing the pixel information and light reservoirs.
 */
class Image
{
public:
    /*
     * Get the reservoir/sample combination for the given pixel at the given depth.
     */
    inline Pixel& At(int a_X, int a_Y)
    {
        return m_Pixels[a_X][a_Y];
    }

    inline const Pixel& At(int a_X, int a_Y) const
    {
        return m_Pixels[a_X][a_Y];
    }

private:
    Pixel m_Pixels[SCREEN_WIDTH][SCREEN_HEIGHT];
};

/*
 * Cumulative Distribution Function.
 * T is the type stored.
 */
template<typename T>
class CDF
{
    struct DataEntry
    {
        //Lower and higher bound.
        float lower;
        float higher;

        //The data contained at this entry.
        T data;
    };

public:
    CDF(std::size_t a_ExpectedSize): m_Sum(0)
    {
        m_DataSet.reserve(a_ExpectedSize);
    }
    
    /*
     * Add an entry to the CDF. This invalidates previously queried PDFs!
     */
    void Append(T a_Entry, float a_Weight)
    {
        //Set sum as lower bound, then append value to sum and set as higher bound.
        m_DataSet.emplace_back(DataEntry{m_Sum, (m_Sum + a_Weight), a_Entry});
        m_Sum += a_Weight;
    }

    /*
     * Find the item belonging to the given value, normalized between 0.0 and 1.0.
     * The dataset may not be empty.
     *
     * The found data object is stored in a_Data.
     * The PDF for that object being chosen is stored as a_Pdf.
     */
    void Find(float a_Value, T& a_Data, float& a_Pdf)
    {
        assert(!m_DataSet.empty() && "Dataset cannot be empty when trying to look up in CDF!");
        assert(a_Value >= 0.f && a_Value <= 1.f && "Value has to be normalized between 0 and 1 (inclusive)!");

        //Index is not normalized in the actual set.
        float index = m_Sum * a_Value;

        //Binary search
        auto& entry = BinarySearch(0, m_DataSet.size() - 1, index);

        //Pdf is proportional to all entries in the dataset.
        a_Data = entry.data;
        a_Pdf = (entry.higher - entry.lower) / (m_Sum);
    }

    /*
     * Find a random value in the set based on weight. This uses the default rand() random engine.
     *
     * The found data object is stored in a_Data.
     * The PDF for that object being chosen is stored as a_Pdf.
     */
    void FindRandom(T& a_Data, float& a_Pdf)
    {
        Find(static_cast<float>(rand()) / static_cast<float>(RAND_MAX), a_Data, a_Pdf);
    }

private:
    /*
     * Binary search to find a value in the set.
     */
    DataEntry& BinarySearch(int a_First, int a_Last, float a_Value)
    {
        assert(a_Value >= 0.f && a_Value <= m_Sum && "Binary search key must be within set bounds.");
        assert(a_First >= 0 && a_First <= a_Last);

        //Get the middle element.
        int center = a_First + (a_Last - a_First) / 2;
        auto& element = m_DataSet[center];

        //Element is smaller, so search in the lower half of the data range.
        if(a_Value < element.lower)
        {
            return BinarySearch(a_First, center - 1, a_Value);
        }

        //Bigger, so search in the upper half of the data range.
        if(a_Value > element.higher)
        {
            return BinarySearch(center + 1, a_Last, a_Value);
        }

        //The value lies between the lower and higher bound, so the current element is the right one.
        return element;
    }

private:
    std::vector<DataEntry> m_DataSet;
    float m_Sum;
};

/*
 * Restir algorithm using all the data structures.
 */
class ReSTIR
{
public:

    ReSTIR(RayTracer& a_RayTracer) : m_RayTracer(a_RayTracer) {}

    /*
     * Run ReSTIR for the current frame.
     * A pointer to the lights array with the light count is provided.
     */
    void Run(const std::vector<Light>& a_Lights);

    /*
     * Swap front and back buffer for temporal samples.
     */
    void SwapBuffers();

    /*
     * Set a pixel value.
     */
    void SetPixelData(int a_X, int a_Y, const linalg::aliases::float3& a_Position, const linalg::aliases::float3& a_Normal, float a_Depth, const Material& a_Material);

    /*
     * Get data about a pixel.
     */
    const Pixel& GetPixelData(int x, int y) const
    {
        return m_Images[m_SwapIndex].At(x, y);
    }

private:
    /*
     * Combine multiple reservoirs into the output reservoir of the pixel.
     * Only reservoirs in a_Reservoirs are weighed, so a_Pixels reservoir is not by default!
     * This introduces bias.
     * The weight of the reservoirs has to be updated before they can be merged (UpdateWeight()).
     *
     * The depth determines which reservoir is updated for the pixel.
     *
     * The final reservoir is then stored inside a_Pixel.
     */
    template<size_t S>
    void CombineBiased(Pixel& a_Pixel, std::array<Reservoir*, S>& a_Reservoirs, int a_Count, int a_Depth)
    {
        Reservoir output;
        int sampleCountSum = 0;

        //Iterate over the reservoirs.
        for(int index = 0; index < a_Count; ++index)
        {
            auto& reservoir = *a_Reservoirs[index];

            auto reSampled = ReSample(reservoir.sample, a_Pixel);

            const float weight = static_cast<float>(reservoir.sampleCount) * reservoir.weight * reSampled.solidAnglePdf;

            assert(reSampled.solidAnglePdf >= 0.f);

            output.Update(reSampled, weight);

            sampleCountSum += reservoir.sampleCount;
        }

        //Update the sample 
        output.sampleCount = sampleCountSum;
        output.UpdateWeight();

        assert(output.weight >= 0.f && output.weightSum >= 0.f);

        a_Pixel.lightReservoirs[a_Depth] = output;
    }

    /*
     * Resample a light sample for the given pixel.
     * The modified sample is returned.
     *
     * In ReSTIR, the refined sampling function P^ roughly equal to the unshadowed path contribution; P^ = rho(x)Le(x)G(x).
     * rho(x) is the BSDF factor of the light on the surface. 
     * Le(x) is the light emitted by x.
     * G(x) is the geometry factor of x, which is equal to the area of the light projected onto the hemisphere of the surface being shaded. This can not be negative, and is 0 at least.
     *
     * G(x) has to be normalized to a value between 0 and 1 depending on how much of the hemisphere is covered. This is why G(x) is equal to (1.f / solid angle).
     * The solid angle is equal to Area(x) * (dot(normal(x), lDir).
     */
    LightSample ReSample(const LightSample& a_LightSample, const Pixel& a_Pixel)
    {
        LightSample r = a_LightSample;

        linalg::aliases::float3 pixelToLightDir = a_LightSample.position - a_Pixel.position;                                   //Direction from pixel to light.
        const float lDistance = linalg::length(pixelToLightDir);                                                               //Light distance from pixel.
        pixelToLightDir /= lDistance;                                                                                          //Normalize.
        const float dot1 = std::clamp(linalg::dot(pixelToLightDir, a_Pixel.normal), 0.f, 1.f);           //Lambertian term clamped between 0 and 1. SurfaceN dot ToLight
        const float dot2 = std::clamp(linalg::dot(a_LightSample.normal, -pixelToLightDir), 0.f, 1.f);    //Light normal at sample point dotted with light direction. Invert light dir for this (light to pixel instead of pixel to light)

        //If the light distance from the surface is negligible, count it as an invalid sample because it's likely self-shading.
        if(lDistance <= 0.01f)
        {
            r.solidAnglePdf = 0.f;
            return r;
        }

        //Geometry term G(x).
        const float solidAngle = (dot1 * dot2) / (lDistance * lDistance);

        //BSDF is equal to material color for now.
        const auto& bsdf = a_Pixel.material.color;

        //Light emittance of the light is equal to it's radiance
        const auto& emittance = a_LightSample.radiance;

        //For the light strenghth/bsdf factors, I take the average of each channel to weigh them.
        const linalg::aliases::float3 colorFactor = bsdf * emittance;
        const float averageColor = (colorFactor.x + colorFactor.y + colorFactor.z) / 3.f;

        r.solidAnglePdf = (static_cast<float>(averageColor) * static_cast<float>(solidAngle));

        assert(r.solidAnglePdf >= 0.f);

        return r;
    }

    /*
     * Combine reservoirs without bias.
     */
    template<size_t S>
    void CombineUnbiased(Pixel& a_Pixel, std::array<Pixel*, S>& a_Pixels, int a_Count, int a_Depth)
    {
        Reservoir output;
        int sampleCountSum = 0;

        for(int index = 0; index < a_Count; ++index)
        {
            auto& otherReservoir = (*a_Pixels[index]).lightReservoirs[a_Depth];
            auto reSampled = ReSample(otherReservoir.sample, a_Pixel);

            const float weight = static_cast<float>(otherReservoir.sampleCount) * otherReservoir.weight * reSampled.solidAnglePdf;

            output.Update(reSampled, weight);

            sampleCountSum += otherReservoir.sampleCount;
        }

        output.sampleCount = sampleCountSum;

        //Weigh against other pixels to remove bias from their solid angle by re-sampling.
        int correction = 0;

        for(int index = 0; index < a_Count; ++index)
        {
            auto& otherPixel = *a_Pixels[index];
            const auto reSampled = ReSample(output.sample, otherPixel);

            if(reSampled.solidAnglePdf > 0)
            {
                correction += otherPixel.lightReservoirs[a_Depth].sampleCount;
            }
        }

        //TODO Shadow ray is shot here in ReSTIR to check visibility at every resampled pixel.

        //TODO I don't understand this part fully, but it's in the pseudocode of ReSTIR. Dive into it when I have time.
        const float m = 1.f / std::fmaxf(static_cast<float>(correction), std::numeric_limits<float>::min());
        output.weight = (1.f / std::fmaxf(output.sample.solidAnglePdf, std::numeric_limits<float>::min())) * (m * output.weightSum);

        //Store the output reservoir for the pixel.
        a_Pixel.lightReservoirs[a_Depth] = output;
    }

private:
    //Swapchain between sample buffers.
    bool m_SwapIndex = true;

    //The buffer containing the reservoirs of samples for each pixel.
    //The third buffer is used to store spatial samples temporarily while combining.
    Image m_Images[3];

    RayTracer& m_RayTracer;
};