#pragma once
#include <iostream>
#include <cmath>
#include <array>
#include "Vector3D.h"
#include "Matrix.h"

class Quat
{
public:
    Quat() : q{ 0.0, 0.0, 0.0, 1.0 } {} 

    Quat(double x, double y, double z, double w) : q{ x, y, z, w } {
    }

    Quat operator*(const Quat& q2) const;
    Quat operator*(const double d) const;

    Vector3D rotate(const Vector3D& vec) const;

    Quat conjugate() const;

    Quat inverse() const;

    // Normalize the quaternion
    void normalize();

    // static Quat fromAxisAngle(const Vector3D& axis, double angle);

    // Matrix toRotationMatrix() const;

    friend std::ostream& operator<<(std::ostream& out, const Quat& q);

    double& operator[](size_t index) { return q[index]; }

    const double& operator[](size_t index) const { return q[index]; }

    std::array<double, 4> q; // [x, y, z, w]
private:
    
};
