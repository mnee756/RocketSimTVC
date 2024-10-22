#pragma once
#include "Point3D.h"
#include "Quat.h"
#include "Matrix.h"
#include <vector>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


constexpr double deg2rad(double degrees) {
	return degrees * (M_PI / 180.0);
}

class Engine
{
public:
	Engine(Vector3D gimbalPoint, double length, double maxThrust = 10, double throttle = 0.0, double mass = 0.0, 
		double maxGimbalAngle = deg2rad(30.0), std::vector<double> gimbalAngle = {0.0, 0.0}) :
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
	Matrix getRengine2rocket();
	Vector3D getGimbalPoint() { return m_gimbalPoint; };

	// Setters
	void setThrottle(double t) { m_throttle = t; }
	void setGimbalAngles(const std::vector<double>& angles);

private:
	Vector3D m_gimbalPoint{};
	double m_length{};
	double m_maxThrust{};
	double m_throttle{};
	double m_maxGimbalAngle{};
	double m_mass{};
	Vector3D m_thrust{};
	std::vector<double> m_gimbalAngle{}; //YPR
};