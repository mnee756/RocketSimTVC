#include "Matrix.h"


Matrix::Matrix(int rows, int cols)
    : m_rows(rows), m_cols(cols), M(rows, std::vector<double>(cols, 0.0)) {}

std::vector<double>& Matrix::operator[](int row) {
    if (row < 0 || row >= m_rows) {
        throw std::out_of_range("Row index out of range");
    }
    return M[row];
}

const std::vector<double>& Matrix::operator[](int row) const {
    if (row < 0 || row >= m_rows) {
        throw std::out_of_range("Row index out of range");
    }
    return M[row];
}

Matrix Matrix::inverse() const 
{
    if (m_rows != m_cols) {
        throw std::invalid_argument("Matrix is not square.");
    }

    if (m_rows != 3) {
        throw std::invalid_argument("3x3 Matrices only.");
    }

    double a11 = M[0][0], a12 = M[0][1], a13 = M[0][2];
    double a21 = M[1][0], a22 = M[1][1], a23 = M[1][2];
    double a31 = M[2][0], a32 = M[2][1], a33 = M[2][2];

    double det = a11 * (a22 * a33 - a23 * a32) -
        a12 * (a21 * a33 - a23 * a31) +
        a13 * (a21 * a32 - a22 * a31);

    if (det == 0) {
        throw std::invalid_argument("Matrix is singular.");
    }

    Matrix inv(3, 3);
    inv.M[0][0] = (a22 * a33 - a23 * a32) / det;
    inv.M[0][1] = (a13 * a32 - a12 * a33) / det;
    inv.M[0][2] = (a12 * a23 - a13 * a22) / det;

    inv.M[1][0] = (a23 * a31 - a21 * a33) / det;
    inv.M[1][1] = (a11 * a33 - a13 * a31) / det;
    inv.M[1][2] = (a13 * a21 - a11 * a23) / det;

    inv.M[2][0] = (a21 * a32 - a22 * a31) / det;
    inv.M[2][1] = (a12 * a31 - a11 * a32) / det;
    inv.M[2][2] = (a11 * a22 - a12 * a21) / det;

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
                result[i][j] += M[i][k] * other[k][j];
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
        M[0][0] * vec.getX() + M[0][1] * vec.getY() + M[0][2] * vec.getZ(),
        M[1][0] * vec.getX() + M[1][1] * vec.getY() + M[1][2] * vec.getZ(),
        M[2][0] * vec.getX() + M[2][1] * vec.getY() + M[2][2] * vec.getZ()
    };
}

Matrix Matrix::transpose() {
    Matrix result(m_cols, m_rows); // The transposed matrix has the number of rows and columns swapped
    for (int i = 0; i < m_rows; ++i) {
        for (int j = 0; j < m_cols; ++j) {
            result[j][i] = M[i][j]; // Swap row and column indices
        }
    }
    return result;
}

// Perform R1 rotation, about z
Matrix R1(double a)
{
    Matrix A(3,3);
    double c1 = cos(a);
    double s1 = sin(a);
    A.M[2][2] = 1.0;
    A.M[0][0] = c1;
    A.M[0][1] = -s1;
    A.M[1][0] = s1;
    A.M[1][1] = c1;
    return A;
}

// Perform R2 rotation, about y
Matrix R2(double a)
{
    Matrix A(3,3);
    double c2 = cos(a);
    double s2 = sin(a);
    A.M[1][1] = 1.0;
    A.M[0][0] = c2;
    A.M[0][2] = s2;
    A.M[2][0] = -s2;
    A.M[2][2] = c2;
    return A;
}

// Perform R3 rotation, about x
Matrix R3(double a)
{
    Matrix A(3,3);
    double c3 = cos(a);
    double s3 = sin(a);

    A.M[0][0] = 1.0;
    A.M[1][1] = c3;
    A.M[1][2] = -s3;
    A.M[2][1] = s3;
    A.M[2][2] = c3;
    return A;
}

std::ostream& operator<<(std::ostream& out, const Matrix& A)
{
    for (int i = 0; i < A.m_rows; ++i) {
        for (int j = 0; j < A.m_cols; ++j) {
            out << A[i][j] << " ";
        }
        out << '\n';  // New line for each row
    }
    return out;;
}
