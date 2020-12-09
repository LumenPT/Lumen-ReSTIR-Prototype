#pragma once
#include "linalg.h"

struct Material
{
	Material() : emissive(false), color(1.f, 1.f, 1.f) {}
	Material(bool a_Emissive, const linalg::aliases::float3& a_Color) : emissive(a_Emissive), color(a_Color){}

	bool emissive;	//True if giving off light. If true, color is the irradiance.
    linalg::aliases::float3 color;
};

struct HitInfo
{
	HitInfo() : distance(-1.f) {}

	float distance;
    linalg::aliases::float3 normal;
	Material material;
};

struct Ray
{
    linalg::aliases::float3 origin;
    linalg::aliases::float3 direction;
};

class Sphere
{
public:
	Sphere(float a_Radius, const linalg::aliases::float3& a_Center, const Material& a_Material) : m_Radius(a_Radius), m_Material(a_Material), m_Center(a_Center)
	{
	    
	}

	Material& GetMaterial()
	{
		return m_Material;
	}

	const Material& GetMaterial() const
	{
		return m_Material;
	}

    linalg::aliases::float3 GetCenter() const
	{
		return m_Center;
	}

	float GetRadius() const
	{
		return m_Radius;
	}

	/*
	 * Get the distance along the ray at which intersection occurs.
	 */
	HitInfo Intersects(const Ray& a_Ray) const
	{
		HitInfo info;
		const linalg::aliases::float3 sphereToRay = a_Ray.origin - m_Center;

		const float a = 1.f;
		const float b = linalg::dot(a_Ray.direction, sphereToRay) * 2.f;
		const float c = linalg::dot(sphereToRay, sphereToRay) - (m_Radius * m_Radius);

		const float discriminant = (b * b) - (4.f * a * c);

		if (discriminant <= 0)
		{
			info.distance = -1.f;
			return info;
		}

		const float twoA = 2.f * a;
		const float root = sqrt(discriminant) / twoA;
		const float sum1 = -b / twoA;
		float intersection0 = sum1 + root;
		float intersection1 = sum1 - root;

		if (intersection0 > intersection1)
		{
			const float temp = intersection0;
			intersection0 = intersection1;
			intersection1 = temp;
		}

		if (intersection0 < 0.005f)
		{
			intersection0 = intersection1;

			if (intersection1 < 0.005f)
			{
				info.distance = -1.f;
				return info;
			}
		}

        const linalg::aliases::float3 position = a_Ray.origin + (a_Ray.direction * intersection0);

		info.distance = intersection0;
		info.normal = linalg::normalize(position - m_Center);
		info.material = m_Material;
	    return info;
	}

private:
	float m_Radius;
	Material m_Material;
    linalg::aliases::float3 m_Center;
};
