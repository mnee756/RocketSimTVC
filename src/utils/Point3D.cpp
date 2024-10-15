#include "Point3D.h"

Point3D::Point3D(double x, double y, double z) :
	m_x{ x }, m_y{ y }, m_z{ z }
{
}

std::ostream& operator<<(std::ostream& out, Point3D pt)
{
	out << "Point3D(" << pt.m_x << ", " << pt.m_y << ", " << pt.m_z << ')';
	return out;
}

