#include "pose/matrix_vector_operators.hpp"

#include <sstream>

namespace rip
{
    namespace navigation
    {
        namespace pose
        {

            Vector operator*(const Matrix& m, const Vector& v)
            {
                if(m.cols() != v.order())
                {
                    std::stringstream error_message;
                    error_message << "Matrix and Vector sizes not compatible " << "(" << m << " * " << v << ")";
                    throw std::runtime_error(error_message.str());
                }
                std::vector<double> result(m.rows());
                for(std::size_t row = 0; row < m.rows(); row ++)
                {
                    double value = 0.0;
                    for(std::size_t col = 0; col < m.cols(); col ++)
                    {
                        value += m.at(row, col) * v[col];
                    }
                    result[row] = value;
                }
                return Vector(result);
            }

            Vector operator*(const Vector& v, const Matrix& m)
            {
                if(v.order() != m.rows())
                {
                    throw std::runtime_error("Matrix size is not compatible with Vector size.");
                }
                std::vector<double> result(v.order());
                for(std::size_t col = 0; col < m.cols(); col ++)
                {
                    for(std::size_t row = 0; row < m.rows(); row ++)
                    {
                        result[row] += v[row] * m.at(row, col);
                    }
                }
                return Vector(result);

            }

            Matrix operator*(double scalar, const Matrix& m)
            {
                const std::vector<double>& ma = m.m_m;
                std::vector<double> a(ma.size());
                for(std::size_t i = 0; i < a.size(); i ++)
                {
                    a[i] = ma[i] * scalar;
                }
                return Matrix(m.rows(), m.cols(), a);

            }

            double operator*(const Vector& v1, const Vector& v2)
            {
                return v1.dot(v2);
            }
        }
    }
}