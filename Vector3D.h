#pragma once
#include <iostream>

class Vector3D
{
public:
	Vector3D(double x, double y, double z);
	double magnitude();
	double dot(Vector3D vec);
	Vector3D cross(Vector3D vec);
	
	// Overloaded Operators
	Vector3D operator/(double scalar);
	Vector3D operator-() const; // Negation operator
	Vector3D operator+(Vector3D vec);
	Vector3D operator-(Vector3D vec);
	bool operator==(const Vector3D& vec) const;
	friend std::ostream& operator<<(std::ostream& out, Vector3D vec);

private:
	double m_x;
	double m_y;
	double m_z;
};