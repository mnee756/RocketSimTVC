#pragma once
#include "Point3D.h"
#include "Quat.h"
#include <array>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using Matrix3x3 = std::array<std::array<double, 3>, 3>;

constexpr double deg2rad(double degrees) {
	return degrees * (M_PI / 180.0);
}

class Engine
{
public:
	Engine(Vector3D gimbalPoint, double length, double maxThrust = 10, double throttle = 0.0, double mass = 0.0, 
		double maxGimbalAngle = deg2rad(10.0), std::array<double, 3> gimbalAngle = {0.0, 0.0, 0.0}) :
		m_gimbalPoint(gimbalPoint),
		m_length(length),
		m_maxThrust(maxThrust),
		m_throttle(throttle),
		m_mass(mass),
		m_maxGimbalAngle(maxGimbalAngle),
		m_gimbalAngle(gimbalAngle) {}
	// Getters
	double getMass() const { return m_mass; }
	Vector3D getThrust();
	Matrix3x3 getRengine2rocket();

	// Setters
	void setThrottle(double t) { m_throttle = t; }
	// need some kind of setter for gimbal angles
private:
	Vector3D m_gimbalPoint{};
	double m_length{};
	double m_maxThrust{};
	double m_throttle{};
	double m_maxGimbalAngle{};
	double m_mass{};
	Vector3D m_thrust{};
	std::array<double, 3> m_gimbalAngle{}; //YPR
};