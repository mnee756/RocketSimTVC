#pragma once
#include <vector>
#include "Engine.h"

struct RocketState
{
	Vector3D pos{};
	Vector3D vel{};
	Vector3D accel{};
	Vector3D ang{};
	Vector3D angVel{};
	Vector3D angAccel{};

	RocketState()
		: pos{ 0.0, 0.0, 0.0 },
		vel{ 0.0, 0.0, 0.0 },
		accel{ 0.0, 0.0, 0.0 },
		ang{ 0.0, 0.0, 0.0 },
		angVel{ 0.0, 0.0, 0.0 },
		angAccel{ 0.0, 0.0, 0.0 } {}
};

class Rocket
{
public:
	Rocket(double length, double radius, double mass)
		: m_length{ length },
		m_radius{ radius },
		m_mass{ mass },
		m_cg{ 0.0, 0.0, length / 2 },
		m_engines{}
	{
		initEngines(m_radius);
		m_inertia[0][0] = (1.0 / 12.0) * m_mass * (3 * m_radius * m_radius + m_length * m_length);
		m_inertia[1][1] = (1.0 / 12.0) * m_mass * (3 * m_radius * m_radius + m_length * m_length);
		m_inertia[2][2] = (1.0 / 2.0) * m_mass * m_radius * m_radius;
	}
	void dynamics(double dt);

	void printState() const {
		std::cout << "Position: " << m_state.pos << std::endl;
		std::cout << "Velocity: " << m_state.vel << std::endl;
		std::cout << "Acceleration: " << m_state.accel << std::endl;
		std::cout << "Angular Position: " << m_state.ang << std::endl;
		std::cout << "Angular Velocity: " << m_state.angVel << std::endl;
		std::cout << "Angular Acceleration: " << m_state.angAccel << std::endl;
	}

private:
	void initEngines(double radius);
	double m_length{};
	double m_radius{};
	double m_mass{};
	Matrix m_inertia{};
	Vector3D m_cg{};
	RocketState m_state{};
	std::vector<Engine> m_engines{};
	std::vector <RocketState> m_data{};
};

