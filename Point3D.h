#pragma once
#include <iostream>

class Point3D
{
public:
	Point3D() = default;
	Point3D(double x, double y, double z);
	friend std::ostream& operator<<(std::ostream& out, Point3D pt);
	
private:
	double m_x;
	double m_y;
	double m_z;
};