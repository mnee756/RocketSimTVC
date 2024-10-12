#include "Vector3D.h"
#include <cmath>

Vector3D::Vector3D(double x, double y, double z) :
	m_x{ x }, m_y{ y }, m_z{ z }
{
}

double Vector3D::magnitude()
{
	return std::sqrt(m_x * m_x + m_y * m_y + m_z * m_z);
}

double Vector3D::dot(Vector3D vec)
{
	return (m_x * vec.m_x + m_y * vec.m_y + m_z * vec.m_z);
}

Vector3D Vector3D::cross(Vector3D vec)
{
	return Vector3D{ 
		m_y * vec.m_z - m_z * vec.m_y, 
		m_z * vec.m_x - m_x * vec.m_z, 
		m_x * vec.m_y - m_y * vec.m_x 
	};
}

Vector3D Vector3D::operator/(double scalar)
{
	return Vector3D{ m_x / scalar, m_y / scalar, m_z / scalar };
}


Vector3D Vector3D::operator-() const {
	return Vector3D{ -m_x, -m_y, -m_z };
}

Vector3D Vector3D::operator+(Vector3D vec)
{
	return Vector3D{ m_x + vec.m_x, m_y + vec.m_y, m_z + vec.m_z };
}

Vector3D Vector3D::operator-(Vector3D vec)
{
	return *this + (-vec);
}

bool Vector3D::operator==(const Vector3D& vec) const {
	return m_x == vec.m_x && m_y == vec.m_y && m_z == vec.m_z;
}

std::ostream& operator<<(std::ostream& out, Vector3D vec)
{
	out << "Vector3D(" << vec.m_x << ", " << vec.m_y << ", " << vec.m_z << ')';
	return out;
}

