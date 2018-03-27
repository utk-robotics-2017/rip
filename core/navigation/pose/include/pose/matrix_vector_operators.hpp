#ifndef MATRIX_VECTOR_OPERATORS_HPP
#define MATRIX_VECTOR_OPERATORS_HPP

#include "pose/vector.hpp"
#include "pose/matrix.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pose
        {
            Vector operator*(const Matrix& m, const Vector& v);
            Vector operator*(const Vector& v, const Matrix& m);
            Matrix operator*(double scalar, const Matrix& m);
            double operator*(const Vector& v1, const Vector& v2);
        }
    }
}

#endif //MATRIX_VECTOR_OPERATORS_HPP