#ifndef GAUSSIAN_MOMENT_HPP
#define GAUSSIAN_MOMENT_HPP

#include <math.h>
#include <memory>
#include <iostream>

#include "../random_distribution.hpp"
#include "../data/vector.hpp"
#include "../data/matrix.hpp"
#include "../data/matrix_vector_operators.hpp"


namespace rip
{
    namespace navigation
    {
        namespace localization
        {
            namespace probability
            {
                template <typename X>
                class GaussianMoment : public RandomDistribution<X>
                {
                public:
                    static double fn(double x, double mean, double variance)
                    {
                        double A = sqrt(1.0 / (2 * M_PI * variance));
                        double dx = x - mean;
                        double x2 = dx * dx;
                        double e = exp(-0.5 * x2 / variance);
                        return A * e;
                    }
                    static double fn(const Vector& x, const Vector& mean, const Matrix& covariance, const Matrix& inverse_covariance)
                    {
                        double A = sqrt(1.0 / ((2 * M_PI) * covariance).determinant());
                        Vector dx = x - mean;
                        double e = exp(-(dx * inverse_covariance * dx));
                        return A * e;
                    }
                    static double fn(const Vector& x, const Vector& mean, const Matrix& covariance)
                    {
                        return fn(x, mean, covariance, covariance.inverse());
                    }

                    GaussianMoment(X mean, Matrix covariance)
                            : m_mean(std::move(mean))
                            , m_covariance(std::move(covariance))
                    {
                    }

                    GaussianMoment(GaussianMoment<X>&& that)
                            : m_mean(std::move(that.m_mean))
                            , m_covariance(std::move(that.m_covariance))
                    {
                    }

                    GaussianMoment<X>& operator=(GaussianMoment<X>&& that) {
                        this->m_mean = std::move(that.m_mean);
                        this->m_covariance = std::move(that.m_covariance);
                        this->m_inverse_covariance = std::move(that.m_inverse_covariance);
                        this->m_sqrt_covariance = std::move(that.m_sqrt_covariance);
                        this->m_is_valid_inverse = m_is_valid_inverse;
                        this->m_is_valid_sqrt = m_is_valid_sqrt;
                        return *this;
                    }

                    virtual ~GaussianMoment()
                    {
                    }

                    double probabilityOf(const X& x) const override
                    {
                        return fn(x.aliasVector(), mean().aliasVector(), covariance(), inverseCovariance());
                    }

                    X sample(std::default_random_engine* generator) const override {
                        std::normal_distribution<double> n01(0, 1);
                        Vector sample(order());
                        for (std::size_t i = 0; i < sample.order(); i++) {
                            // need a N(0,1) here
                            sample[i] = n01(*generator);
                        }
                        X result;
                        result.set(mean() + sqrtCovariance() * sample);
                        return result;
                    }

                    std::size_t order() const {
                        return mean().order();
                    }

                    const X& mean() const {
                        return m_mean;
                    }

                    const Matrix& covariance() const {
                        return m_covariance;
                    }

                    const Matrix& inverseCovariance() const {
                        if (!m_is_valid_inverse) {
                            m_inverse_covariance = m_covariance.inverse();
                            m_is_valid_inverse = true;
                        }
                        return m_inverse_covariance;
                    }

                    const Matrix& sqrtCovariance() const {
                        if (!m_is_valid_sqrt) {
                            m_sqrt_covariance = covariance().sqrt();
                            m_is_valid_sqrt = true;
                        }
                        return m_sqrt_covariance;
                    }

                private:
                    X m_mean;
                    Matrix m_covariance;
                    // These values are mutable because they are cached derivations of covariance
                    mutable Matrix m_inverse_covariance;
                    mutable Matrix m_sqrt_covariance;
                    mutable bool m_is_valid_inverse = false;
                    mutable bool m_is_valid_sqrt = false;
                };
            }
        }
    }
}


#endif // GAUSSIAN_MOMENT_HPP
