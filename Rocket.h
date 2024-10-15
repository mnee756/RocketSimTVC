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
		  m_ang{ 0.0, 0.0, 0.0 },
		  m_engines {}
	{
		initEngines(m_radius);
		m_inertia[0][0] = (1.0 / 12.0) * m_mass * (3 * m_radius * m_radius + m_length * m_length);
		m_inertia[1][1] = (1.0 / 12.0) * m_mass * (3 * m_radius * m_radius + m_length * m_length);
		m_inertia[2][2] = (1.0 / 2.0) * m_mass * m_radius * m_radius;
	}
	void dynamics(double dt);

	void printState() const {
		std::cout << "Position: " << m_pos << std::endl;
		std::cout << "Velocity: " << m_vel << std::endl;
		std::cout << "Acceleration: " << m_accel << std::endl;
		std::cout << "Angular Position: " << m_ang << std::endl;
		std::cout << "Angular Velocity: " << m_angVel << std::endl;
		std::cout << "Angular Acceleration: " << m_angAccel << std::endl;
	}

private:
	void initEngines(double radius);
	double m_length{};
	double m_radius{};
	double m_mass{};
	Matrix m_inertia{};
	Vector3D m_cg{};
	Vector3D m_pos{};
	Vector3D m_vel{};
	Vector3D m_accel{};
	Vector3D m_ang{};
	Vector3D m_angVel{};
	Vector3D m_angAccel{};
	std::vector<Engine> m_engines{};
};