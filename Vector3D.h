#pragma once
#include <iostream>

class Vector3D
{
public:
	Vector3D() = default;
	Vector3D(double x, double y, double z);
	double getX() const { return m_x; }
	double getY() const { return m_y; }
	double getZ() const { return m_z; }
	void setX(double x) { m_x = x; }
	void setY(double y) { m_y = y; }
	void setZ(double z) { m_z = z; }
	double magnitude();
	double dot(Vector3D vec);
	Vector3D cross(Vector3D vec);
	
	// Overloaded Operators
	Vector3D operator/(double scalar);
	Vector3D operator-() const; // Negation operator
	Vector3D operator+(Vector3D vec);
	Vector3D operator-(Vector3D vec);
	Vector3D operator+=(Vector3D vec);
	bool operator==(const Vector3D& vec) const;
	friend std::ostream& operator<<(std::ostream& out, Vector3D vec);

private:
	double m_x;
	double m_y;
	double m_z;
};

Vector3D operator*(const Matrix3x3& matrix, const Vector3D& vec) {
	return Vector3D{
		matrix[0][0] * vec.getX() + matrix[0][1] * vec.getY() + matrix[0][2] * vec.getZ(),
		matrix[1][0] * vec.getX() + matrix[1][1] * vec.getY() + matrix[1][2] * vec.getZ(),
		matrix[2][0] * vec.getX() + matrix[2][1] * vec.getY() + matrix[2][2] * vec.getZ()
	};
}
