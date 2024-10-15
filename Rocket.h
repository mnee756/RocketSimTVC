#pragma once
#include <vector>
#include "Engine.h"

class Rocket
{
public:
	Rocket(double length, double radius, double mass)
		: m_length{ length }, 
		  m_radius{ radius }, 
		  m_mass{ mass }, 
		  m_cg{ 0.0, 0.0, length / 2 },
		  m_pos{ 0.0, 0.0, 0.0 },
		  m_theta{ 0.0, 0.0, 0.0 },
		  m_engines {}
	{
		initEngines(m_radius);
	}
	void dynamics(double dt);

private:
	void initEngines(double radius);
	double m_length{};
	double m_radius{};
	double m_mass{};
	Vector3D m_cg{};
	Vector3D m_pos{};
	Vector3D m_theta{};
	std::vector<Engine> m_engines{};
};