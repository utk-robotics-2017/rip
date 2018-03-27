#ifndef MATRIX_HPP
#define MATRIX_HPP

#include <cstring>
#include <vector>
#include <ostream>

namespace rip
{
    namespace navigation
    {
        namespace pose
        {
            class Vector;

            class Matrix
            {
            public:
                static Matrix identity(std::size_t order);

                Matrix();
                Matrix(std::size_t rows, std::size_t cols);
                Matrix(std::size_t rows, std::size_t cols, std::vector<double> values);
                Matrix(std::size_t rows, std::size_t cols, std::vector<double> values, bool row_based);
                Matrix(const Matrix& rhs);
                Matrix(Matrix&& rhs);

                virtual ~Matrix();

                std::size_t rows() const;
                std::size_t cols() const;
                std::size_t order() const;

                double at(std::size_t row, std::size_t col) const;
                Vector column(std::size_t col) const;

                Matrix& operator=(Matrix&& rhs);
                Matrix& operator+=(const Matrix& rhs);
                Matrix operator+(const Matrix& rhs) const;
                Matrix operator-(const Matrix& rhs) const;
                Matrix operator*(const Matrix& rhs) const;

                Matrix transpose() const;
                Matrix inverse() const;
                Matrix sqrt() const;

                double determinant() const;
                void scaleRow(std::size_t r, double k);
                void combineRow(std::size_t src_row, std::size_t dst_row, double k);

                Matrix LUDecompositionByGE(int* p_swap_count, std::vector< std::size_t >* p_pivot) const;
                Matrix roundSymmetry();

            private:
                std::size_t getIndex(std::size_t row, std::size_t col) const;
                static void initPivots(std::size_t n, std::vector<std::size_t>* p_pivot);
                double partialPivot(std::vector<std::size_t>* p_pivot, std::size_t k, int* p_swap_count) const;
                bool isSymmetric() const;
                Matrix choleskyDecomposition() const;

                std::size_t m_rows;
                std::size_t m_cols;
                std::vector<double> m_m;
                bool m_row_based = true;

                friend Matrix operator*(double scalar, const Matrix& m);
            };

            std::ostream& operator<<(std::ostream& os, const Matrix& m);
        }
    }
}

#endif //MATRIX_HPP
