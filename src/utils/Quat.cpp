#include "Quat.h"

void Quat::normalize() 
{
     double norm = std::sqrt(m_x * m_x + m_y * m_y + m_z * m_z + m_w * m_w);
     if (norm > 0.0) {
         m_x /= norm;
         m_y /= norm;
         m_z /= norm;
         m_w /= norm;
     }
}

Quat Quat::operator*(const Quat& q) const {
    return Quat(
        m_x * q.m_w + m_y * q.m_z - m_z * q.m_y + m_w * q.m_x,
        m_z * q.m_x - m_y * q.m_w + m_x * q.m_y + m_w * q.m_z,
        m_y * q.m_x + m_x * q.m_w - m_z * q.m_z + m_w * q.m_y,
        m_w * q.m_w - m_x * q.m_x - m_y * q.m_y - m_z * q.m_z
    );
}


void Quat::rotate(Vector3D& vec) const {
    Quat qv(vec.getX(), vec.getY(), vec.getZ(), 0.0);  // Vector as quaternion
    Quat qr = (*this) * qv * conjugate(); // q * v * q*
    vec.setX(qr.m_x);  // Rotated vector components
    vec.setY(qr.m_y);
    vec.setZ(qr.m_z);
}

Quat Quat::conjugate() const {
    return Quat(-m_x, -m_y, -m_z, m_w);
}


std::ostream& operator<<(std::ostream& out, Quat q)
{
    out << "Quaternion(" << q.m_x << ", " << q.m_y << ", " << q.m_z << ", " << q.m_w << ")\n";
    return out;
}