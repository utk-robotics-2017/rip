#include "pose/matrix.hpp"
#include "pose/vector.hpp"
#include <algorithm>

namespace rip
{
    namespace navigation
    {
        namespace pose
        {
            Matrix::Matrix() : m_rows(0), m_cols(0), m_m(), m_row_based(true)
            {
            }

            Matrix::Matrix(std::size_t rows, std::size_t cols) : m_rows(rows), m_cols(cols), m_m(rows * cols),
                                                                 m_row_based(true)
            {
            }

            Matrix::Matrix(std::size_t rows, std::size_t cols, std::vector<double> values) : m_rows(rows), m_cols(cols),
                                                                                             m_m(values),
                                                                                             m_row_based(true)
            {
            }

            Matrix::Matrix(std::size_t rows, std::size_t cols, std::vector<double> values, bool row_based) : m_rows(
                    rows), m_cols(cols), m_m(values), m_row_based(row_based)
            {
            }

            Matrix::Matrix(const Matrix& that) : m_rows(that.m_rows), m_cols(that.m_cols), m_m(that.m_m),
                                                 m_row_based(that.m_row_based)
            {
            }

            Matrix::Matrix(Matrix&& that) : m_rows(that.m_rows), m_cols(that.m_cols), m_m(std::move(that.m_m)),
                                            m_row_based(that.m_row_based)
            {
            }

            Matrix::~Matrix()
            {
            }

            std::size_t Matrix::rows() const
            {
                return m_rows;
            }

            std::size_t Matrix::cols() const
            {
                return m_cols;
            }

            std::size_t Matrix::order() const
            {
                if(m_rows != m_cols)
                {
                    throw std::runtime_error("Matrix must be square.");
                }
                return m_rows;
            }

            double Matrix::at(std::size_t row, std::size_t col) const
            {
                return m_m[getIndex(row, col)];
            }

            Vector Matrix::column(std::size_t col) const
            {
                Vector result;
                std::vector<double> a(m_rows);
                if(m_row_based)
                {
                    for(std::size_t i = 0; i < a.size(); i ++)
                    {
                        a[i] = at(i, col);
                    }
                }else
                {
                    std::size_t col_start = getIndex(0, col);
                    std::size_t col_end = col_start + a.size();
                    std::copy(m_m.begin() + col_start, m_m.begin() + col_end, a.begin());
                }
                return Vector(a);
            }

            Matrix& Matrix::operator=(Matrix&& that)
            {
                m_rows = that.m_rows;
                m_cols = that.m_cols;
                m_m = std::move(that.m_m);
                m_row_based = that.m_row_based;
                return *this;
            }

            Matrix& Matrix::operator+=(const Matrix& that)
            {
                if(m_rows != that.m_rows || m_cols != that.m_cols)
                {
                    throw std::runtime_error("Matrix sizes not compatible.");
                }

                if(m_row_based == that.m_row_based)
                {
                    for(std::size_t i = 0; i < m_m.size(); i ++)
                    {
                        m_m[i] += that.m_m[i];
                    }
                }else
                {
                    for(std::size_t row = 0; row < rows(); row ++)
                    {
                        for(std::size_t col = 0; col < cols(); col ++)
                        {
                            std::size_t this_index = getIndex(row, col);
                            std::size_t that_index = that.getIndex(row, col);
                            m_m[this_index] += that.m_m[that_index];
                        }
                    }
                }

                return *this;
            }

            Matrix Matrix::operator+(const Matrix& that) const
            {
                if(m_rows != that.m_rows || m_cols != that.m_cols || m_row_based != that.m_row_based)
                {
                    throw std::runtime_error("Matrix sizes not compatible.");
                }
                std::vector<double> m(m_m.size());
                for(std::size_t i = 0; i < m_m.size(); i ++)
                {
                    m[i] = m_m[i] + that.m_m[i];
                }
                return Matrix(m_rows, m_cols, m, m_row_based);
            }

            Matrix Matrix::operator-(const Matrix& that) const
            {
                if(m_rows != that.m_rows || m_cols != that.m_cols || m_row_based != that.m_row_based)
                {
                    throw std::runtime_error("Matrix sizes not compatible.");
                }
                std::vector<double> m(m_m.size());
                for(std::size_t i = 0; i < m_m.size(); i ++)
                {
                    m[i] = m_m[i] - that.m_m[i];
                }
                return Matrix(m_rows, m_cols, m, m_row_based);
            }

            Matrix Matrix::operator*(const Matrix& that) const
            {
                if(m_cols != that.m_rows)
                {
                    throw std::runtime_error("Matrix sizes not compatible.");
                }
                Matrix result(m_rows, that.m_cols);
                for(std::size_t col = 0; col < that.m_cols; col ++)
                {
                    for(std::size_t row = 0; row < m_rows; row ++)
                    {
                        for(std::size_t k = 0; k < m_cols; k ++)
                        {
                            result.m_m[result.getIndex(row, col)] += at(row, k) * that.at(k, col);
                        }
                    }
                }
                return result;
            }

            Matrix Matrix::transpose() const
            {
                return Matrix(m_cols, m_rows, m_m, ! m_row_based);
            }

            Matrix Matrix::inverse() const
            {
                if(m_rows != m_cols)
                {
                    throw std::runtime_error("Matrix must be square.");
                }
                // set up mean as identity
                Matrix result(m_rows, m_cols);
                for(std::size_t i = 0; i < m_cols; i ++)
                {
                    result.m_m[result.getIndex(i, i)] = 1.0;
                }

                Matrix working(m_rows, m_cols, m_m);  // creates a copy of m_
                // Eliminate L
                for(std::size_t col = 0; col < m_cols; col ++)
                {
                    double diag = working.at(col, col);
                    for(std::size_t row = col + 1; row < m_rows; row ++)
                    {
                        double target = working.at(row, col);
                        double a = - target / diag;
                        working.combineRow(col, row, a);
                        result.combineRow(col, row, a);
                    }
                    working.scaleRow(col, 1.0 / diag);
                    result.scaleRow(col, 1.0 / diag);
                }
                // Eliminate U
                for(std::size_t col = m_cols - 1; col >= 1; col --)
                {
                    double diag = working.at(col, col);  // 1.0
                    for(std::size_t row_plus_one = col; row_plus_one > 0; row_plus_one --)
                    {
                        std::size_t row = row_plus_one - 1;
                        double target = working.at(row, col);
                        double a = - target / diag;
                        working.combineRow(col, row, a);
                        result.combineRow(col, row, a);
                    }
                }
                return result;
            }

            Matrix Matrix::sqrt() const
            {
                // The matrix square root should be calculated using numerically efficient and stable methods such as the Cholesky decomposition
                if(m_rows != m_cols)
                {
                    throw std::runtime_error("Matrix must be square.");
                }
                return choleskyDecomposition();
            }

            double Matrix::determinant() const
            {
                if(rows() != cols())
                {
                    throw std::runtime_error("Matrix must be square.");
                }
                switch(rows())
                {
                    case 0:
                        return 1;
                    case 1:
                        return m_m[0];
                    case 2:
                    {
                        return m_m[0] * m_m[3] - m_m[1] * m_m[2];
                    }
                    case 3:
                    {
                        double a = at(0, 0);
                        double b = at(0, 1);
                        double c = at(0, 2);
                        double d = at(1, 0);
                        double e = at(1, 1);
                        double f = at(1, 2);
                        double g = at(2, 0);
                        double h = at(2, 1);
                        double i = at(2, 2);
                        return a * e * i + b * f * g + c * d * h - a * f * h - b * d * i - c * e * g;
                    }
                    default:
                    {
                        int swapCount;
                        std::vector<std::size_t> pivot;
                        Matrix LU = LUDecompositionByGE(&swapCount, &pivot);
                        double result = 1.0;
                        for(std::size_t k = 0; k < LU.order(); k ++)
                        {
                            result *= LU.at(pivot[k], k);
                        }
                        if(swapCount % 2 == 1)
                        {
                            result = - result;
                        }
                        return result;
                    }
                }
            }

            void Matrix::scaleRow(std::size_t r, double k)
            {
                for(std::size_t c = 0; c < m_cols; c ++)
                {
                    m_m[getIndex(r, c)] = k * at(r, c);
                }
            }

            void Matrix::combineRow(std::size_t srcRow, std::size_t dstRow, double k)
            {
                for(std::size_t c = 0; c < m_cols; c ++)
                {
                    m_m[getIndex(dstRow, c)] = at(dstRow, c) + k * at(srcRow, c);
                }
            }

            Matrix Matrix::LUDecompositionByGE(int* p_swap_count, std::vector<std::size_t>* p_pivot) const
            {
                if(m_rows != m_cols)
                {
                    throw std::runtime_error("Matrix must be square.");
                }
                std::vector<std::size_t>& pivot = *p_pivot;
                *p_swap_count = 0;
                std::size_t n = rows();

                // Make a copy of the matrix.
                // We never change the original matrix because the backing array may be reused.
                std::vector<double> a(m_m);
                Matrix m(n, n, a);

                initPivots(n, p_pivot);
                for(std::size_t k = 0; k < n - 1; k ++)
                {
                    double maxValue = m.partialPivot(p_pivot, k, p_swap_count);
                    if(maxValue == 0)
                    {
                        throw std::runtime_error("Matrix is singular.");
                    }
                    double m_kk = m.at(pivot[k], k);
                    for(std::size_t i = k + 1; i < n; i ++)
                    {
                        std::size_t ik = m.getIndex(pivot[i], k);
                        m.m_m[ik] /= m_kk;
                    }
                    for(std::size_t i = k + 1; i < n; i ++)
                    {
                        double m_ik = m.at(pivot[i], k);
                        for(std::size_t j = k + 1; j < n; j ++)
                        {
                            double m_kj = m.at(pivot[k], j);
                            std::size_t ij = m.getIndex(pivot[i], j);
                            m.m_m[ij] -= m_ik * m_kj;
                        }
                    }
                }
                if(m.at(pivot[n - 1], n - 1) == 0)
                {
                    throw std::runtime_error("Matrix is singular.");
                }
                return m;
            }

            Matrix Matrix::roundSymmetry()
            {
                std::vector<double> a(m_m.size());
                for(std::size_t i = 0; i < m_rows; i ++)
                {
                    std::size_t diagIndex = getIndex(i, i);
                    a[diagIndex] = m_m[diagIndex];
                    for(std::size_t j = i + 1; j < m_cols; j ++)
                    {
                        std::size_t i1 = getIndex(i, j);
                        std::size_t i2 = getIndex(j, i);
                        double mean = (m_m[i1] + m_m[i2]) / 2;
                        a[i1] = a[i2] = mean;
                    }
                }
                return Matrix(m_rows, m_cols, a);
            }

            std::size_t Matrix::getIndex(std::size_t row, std::size_t col) const
            {
                if(m_row_based)
                    return row * m_cols + col;
                else
                    return col * m_rows + row;
            }

            /* static */
            void Matrix::initPivots(std::size_t n, std::vector<std::size_t>* p_pivot)
            {
                std::vector<std::size_t>& pivot = *p_pivot;
                pivot.resize(n);
                for(std::size_t i = 0; i < pivot.size(); i ++)
                {
                    pivot[i] = i;
                }
            }

            double Matrix::partialPivot(std::vector<std::size_t>* p_pivot, std::size_t k, int* p_swap_count) const
            {
                std::vector<std::size_t>& pivot = *p_pivot;
                int& swap_count = *p_swap_count;
                double maxValue = fabs(at(pivot[k], k));
                std::size_t maxIndex = k;
                for(std::size_t i = k + 1; i < pivot.size(); i ++)
                {
                    std::size_t rowIndex = pivot[i];
                    double rowValue = fabs(at(rowIndex, k));
                    if(rowValue > maxValue)
                    {
                        maxValue = rowValue;
                        maxIndex = i;
                    }
                }
                if(maxIndex != k)
                {
                    std::size_t temp = pivot[maxIndex];
                    pivot[maxIndex] = pivot[k];
                    pivot[k] = temp;
                    swap_count ++;
                }
                return maxValue;
            }

            bool Matrix::isSymmetric() const
            {
                if(m_rows != m_cols)
                {
                    return false;
                }
                for(std::size_t i = 1; i < rows(); i ++)
                {
                    for(std::size_t j = 1; j < i; j ++)
                    {
                        if(at(i, j) != at(j, i))
                        {
                            return false;
                        }
                    }
                }
                return true;
            }

            Matrix Matrix::choleskyDecomposition() const
            {
                // returns an upper diagonal matrix
                if(m_rows != m_cols)
                {
                    throw std::runtime_error("Matrix must be square.");
                }
                if(! isSymmetric())
                {
                    throw std::runtime_error("Matrix must be symmetric.");
                }
                std::size_t n = rows();
                std::vector<double> a(m_m);
                for(std::size_t i = 0; i < n; i ++)
                {
                    std::size_t ii = getIndex(i, i);
                    for(std::size_t k = 0; k < i; k ++)
                    {
                        std::size_t ki = getIndex(k, i);
                        a[ii] = a[ii] - a[ki] * a[ki];
                    }
                    if(a[ii] < 0)
                    {
                        throw std::runtime_error("Matrix is not positive definite.");
                    }
                    a[ii] = std::sqrt(a[ii]);
                    for(std::size_t j = i + 1; j < n; j ++)
                    {
                        std::size_t ij = getIndex(i, j);
                        for(std::size_t k = 0; k < i; k ++)
                        {
                            std::size_t ki = getIndex(k, i);
                            std::size_t kj = getIndex(k, j);
                            a[ij] = a[ij] - a[ki] * a[kj];
                        }
                        if(a[ij] != 0)
                            a[ij] = a[ij] / a[ii];
                    }
                }
                // Clear out the lower matrix
                for(std::size_t i = 1; i < n; i ++)
                {
                    for(std::size_t j = 0; j < i; j ++)
                    {
                        std::size_t ij = getIndex(i, j);
                        a[ij] = 0;
                    }
                }
                return Matrix(n, n, a);
            }

            std::ostream& operator<<(std::ostream& os, const Matrix& m)
            {
                os << m.rows() << "x" << m.cols() << ":{";
                for(std::size_t row = 0; row < m.rows(); row ++)
                {
                    if(row > 0)
                        os << ", ";
                    os << "{";
                    for(std::size_t col = 0; col < m.cols(); col ++)
                    {
                        if(col > 0)
                            os << ", ";
                        os << m.at(row, col);
                    }
                    os << "}";
                }
                os << "}";
                return os;

            }
        }
    }
}