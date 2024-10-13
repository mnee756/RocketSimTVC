#pragma once
#include <vector>
#include "Engine.h"

class Rocket
{
public:
	Rocket(double length, double radius, double mass)
		: m_length{ length }, m_radius{ radius }, m_mass{ mass }, m_engines{} 
	{
		initEngines(m_radius);
	}


private:
	void initEngines(double radius);
	double m_length{};
	double m_radius{};
	double m_mass{};
	std::vector<Engine> m_engines{};
};