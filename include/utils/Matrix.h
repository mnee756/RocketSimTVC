#pragma once

#include <vector>
#include <stdexcept>
#include "Vector3D.h"

class Matrix {
public:
    Matrix(int rows = 3, int cols = 3); 
    std::vector<double>& operator[](int row); 
    const std::vector<double>& operator[](int row) const; 
    Matrix inverse() const; 
    Matrix operator*(const Matrix& other) const; 
    Vector3D operator*(const Vector3D& vec) const;
    Matrix transpose();

    std::vector<std::vector<double>> M; 
    friend std::ostream& operator<<(std::ostream& out, const Matrix& A);

private:
    int m_rows; 
    int m_cols; 
    
};

Matrix R1(double a);
Matrix R2(double a);
Matrix R3(double a);