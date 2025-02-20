#pragma once
#include <vector>
#include "Engine.h"
#include "Vector3D.h"
#include "Quat.h"


struct RocketState
{
	Vector3D pos{};
	Vector3D vel{};
	Quat q{};      // Orientation quaternion
	Quat qDot{};   // Time derivative of quaternion (quaternion form of angular velocity)
	Vector3D angVel{};  // Body-frame angular velocity

	RocketState()
		: pos{ 0.0, 0.0, 0.0 },
		vel{ 0.0, 0.0, 0.0 },
		q{ 0.0, 0.0, 0.0, 1.0 },  // Identity quaternion
		qDot{ 0.0, 0.0, 0.0, 0.0 },
		angVel{ 0.0, 0.0, 0.0 } {}

	RocketState(Vector3D position)
		: pos{ position },
		vel{ 0.0, 0.0, 0.0 },
		q{ 0.0, 0.0, 0.0, 1.0 },
		qDot{ 0.0, 0.0, 0.0, 0.0 },
		angVel{ 0.0, 0.0, 0.0 } {}
};

struct Input
{
	std::vector<std::vector<double>> gimbalAngles = { {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0} };
	std::vector<double> throttle{0, 0, 0, 0};

	Input()
		: gimbalAngles(4, std::vector<double>(2, 0.0)), throttle(4, 0.0) {}

	Input(std::vector<std::vector<double>> gAngles, std::vector<double> thr)
		: gimbalAngles(std::move(gAngles)), throttle(std::move(thr)) {}

	friend std::ostream& operator<<(std::ostream& out,Input input)
	{
		out << "Input details:\n";
		for (int i = 0; i < input.throttle.size(); ++i) {
			out << "Throttle[" << i << "]: " << input.throttle[i] << "\n";
		}
		for (int i = 0; i < input.gimbalAngles.size(); ++i) {
			out << "Gimbal angles[" << i << "]: " << input.gimbalAngles[i][0] << ", " << input.gimbalAngles[i][1] << "\n";
		}
		return out;
	}
};

class Rocket
{
public:
	Rocket(double length, double radius, double mass, Vector3D initialPos)
		: m_length{ length },
		m_radius{ radius },
		m_mass{ mass },
		m_cg{ 0.0, 0.0, 2 * length / 3 },
		m_state{initialPos},
		m_engines{}
	{
		initEngines(m_radius);
		m_inertia[0][0] = (1.0 / 12.0) * m_mass * (3 * m_radius * m_radius + m_length * m_length);
		m_inertia[1][1] = (1.0 / 12.0) * m_mass * (3 * m_radius * m_radius + m_length * m_length);
		m_inertia[2][2] = (1.0 / 2.0) * m_mass * m_radius * m_radius;
	}
	
	RocketState dynamics(RocketState state, const Input& input, double dt); // Calculates next state from current state
	void update(const Input& input, double dt);				// Calls dynamics() and writes to state history
	void processInput(const Input& input);
	Vector3D transformToBodyFrame(Vector3D vec, RocketState& state);
	Vector3D transformToWorldFrame(Vector3D vec, RocketState& state);

	void printState() const {
		std::cout << "Position: " << m_state.pos << std::endl;
		std::cout << "Velocity: " << m_state.vel << std::endl;
		std::cout << "Orientation (Quat): " << m_state.q << std::endl;
		std::cout << "Quat Derivative: " << m_state.qDot << std::endl;
		std::cout << "Angular Velocity: " << m_state.angVel << std::endl;
	}


	void plotTrajectory() const;
	RocketState& getState() { return m_state; }

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

