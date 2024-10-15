#include "Matrix.h"


Matrix::Matrix(int rows, int cols)
    : m_rows(rows), m_cols(cols), m_data(rows, std::vector<double>(cols, 0.0)) {}

std::vector<double>& Matrix::operator[](int row) {
    if (row < 0 || row >= m_rows) {
        throw std::out_of_range("Row index out of range");
    }
    return m_data[row];
}

const std::vector<double>& Matrix::operator[](int row) const {
    if (row < 0 || row >= m_rows) {
        throw std::out_of_range("Row index out of range");
    }
    return m_data[row];
}

Matrix Matrix::inverse() const 
{
    if (m_rows != m_cols) {
        throw std::invalid_argument("Matrix is not square.");
    }

    if (m_rows != 3) {
        throw std::invalid_argument("3x3 Matrices only.");
    }

    double a11 = m_data[0][0], a12 = m_data[0][1], a13 = m_data[0][2];
    double a21 = m_data[1][0], a22 = m_data[1][1], a23 = m_data[1][2];
    double a31 = m_data[2][0], a32 = m_data[2][1], a33 = m_data[2][2];

    double det = a11 * (a22 * a33 - a23 * a32) -
        a12 * (a21 * a33 - a23 * a31) +
        a13 * (a21 * a32 - a22 * a31);

    if (det == 0) {
        throw std::invalid_argument("Matrix is singular.");
    }

    Matrix inv(3, 3);
    inv.m_data[0][0] = (a22 * a33 - a23 * a32) / det;
    inv.m_data[0][1] = (a13 * a32 - a12 * a33) / det;
    inv.m_data[0][2] = (a12 * a23 - a13 * a22) / det;

    inv.m_data[1][0] = (a23 * a31 - a21 * a33) / det;
    inv.m_data[1][1] = (a11 * a33 - a13 * a31) / det;
    inv.m_data[1][2] = (a13 * a21 - a11 * a23) / det;

    inv.m_data[2][0] = (a21 * a32 - a22 * a31) / det;
    inv.m_data[2][1] = (a12 * a31 - a11 * a32) / det;
    inv.m_data[2][2] = (a11 * a22 - a12 * a21) / det;

    return inv;
}

// Matrix multiplication
Matrix Matrix::operator*(const Matrix& other) const {
    if (m_cols != other.m_rows) {
        throw std::invalid_argument("Matrix dimensions do not allow multiplication");
    }

    Matrix result(m_rows, other.m_cols);
    for (int i = 0; i < m_rows; ++i) {
        for (int j = 0; j < other.m_cols; ++j) {
            for (int k = 0; k < m_cols; ++k) {
                result[i][j] += m_data[i][k] * other[k][j];
            }
        }
    }
    return result;
}

Vector3D Matrix::operator*(const Vector3D& vec) const {
    if (m_cols != 3) { // Check if the matrix can be multiplied with a 3D vector
        throw std::invalid_argument("Matrix columns must match vector size.");
    }

    return Vector3D{
        m_data[0][0] * vec.getX() + m_data[0][1] * vec.getY() + m_data[0][2] * vec.getZ(),
        m_data[1][0] * vec.getX() + m_data[1][1] * vec.getY() + m_data[1][2] * vec.getZ(),
        m_data[2][0] * vec.getX() + m_data[2][1] * vec.getY() + m_data[2][2] * vec.getZ()
    };
}