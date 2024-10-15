#pragma once
#include <iostream>
#include <cmath>
#include "Vector3D.h"

class Quat
{
public:
    Quat() : m_x{ 0.0 }, m_y{ 0.0 }, m_z { 0.0 }, m_w{ 0.0 } {}

    Quat(double x, double y, double z, double w)
        : m_x(x), m_y(y), m_z(z), m_w(w) {
        normalize();
    }

    void normalize();
    Quat operator*(const Quat& q) const;

    // Rotate a vector using the quaternion
    void rotate(Vector3D& vec) const; 

    Quat conjugate() const; 
    friend std::ostream& operator<<(std::ostream& out, Quat q);
    
private:
    double m_x{};
    double m_y{};
    double m_z{};
    double m_w{};
};