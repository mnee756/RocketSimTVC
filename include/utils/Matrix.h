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


private:
    int m_rows; 
    int m_cols; 
    std::vector<std::vector<double>> m_data; 
};
