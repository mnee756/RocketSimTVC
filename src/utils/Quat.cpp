#include "Quat.h"


void Quat::normalize()
{
    double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (norm > 0.0)
    {
        for (int i = 0; i < 4; ++i)
            q[i] /= norm;
    }
}


Quat Quat::operator*(const Quat& q2) const
{
    // [w, x, y, z] * [w', x', y', z']
    return Quat(
        q[3] * q2.q[0] + q[0] * q2.q[3] + q[1] * q2.q[2] - q[2] * q2.q[1],  // x
        q[3] * q2.q[1] - q[0] * q2.q[2] + q[1] * q2.q[3] + q[2] * q2.q[0],  // y
        q[3] * q2.q[2] + q[0] * q2.q[1] - q[1] * q2.q[0] + q[2] * q2.q[3],  // z
        q[3] * q2.q[3] - q[0] * q2.q[0] - q[1] * q2.q[1] - q[2] * q2.q[2]   // w
    );
}

Quat Quat::operator*(const double d) const
{
    return Quat(q[0] * d, q[1] * d, q[2] * d, q[3] * d);
}

Vector3D Quat::rotate(const Vector3D& vec) const
{
    Quat q_vec(vec.getX(), vec.getY(), vec.getZ(), 0.0);  
    Quat q_conjugate = conjugate();

    Quat q_rotated = (*this) * q_vec * q_conjugate;
    return Vector3D(q_rotated.q[0], q_rotated.q[1], q_rotated.q[2]);
}

Quat Quat::conjugate() const
{
    return Quat(-q[0], -q[1], -q[2], q[3]);
}

Quat Quat::inverse() const
{
    return conjugate(); // For unit quaternions, inverse is the conjugate
}

std::ostream& operator<<(std::ostream& out, const Quat& q)
{
    out << "(" << q.q[0] << ", " << q.q[1] << ", " << q.q[2] << ", " << q.q[3] << ")";
    return out;
}