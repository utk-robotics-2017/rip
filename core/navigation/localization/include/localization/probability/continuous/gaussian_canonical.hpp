#ifndef GAUSSIAN_CANONICAL_HPP
#define GAUSSIAN_CANONICAL_HPP

#include <map>
#include <memory>

#include "../random_distribution.hpp"
#include "../data/vector.hpp"
#include "../data/matrix.hpp"
#include "gaussian_moment.hpp"

namespace rip
{
    namespace navigation
    {
        namespace localization
        {
            namespace probability
            {
                template <typename X>
                class GaussianCanonical : public RandomDistribution<X>
                {
                public:
                    GaussianCanonical(const Vector& xi, const Matrix& omega) :
                        m_information(std::move(xi)),
                        m_precision(std::move(omega)),
                        m_sigma(omega.rows(), omega.cols()),
                        m_mu()
                    {
                    }

                    explicit GaussianCanonical(const GaussianMoment<X>& x) :
                        m_information(x.covariance().Inverse()),
                        m_precision(m_information * x.mean()),
                        m_mu(x.mean()),
                        m_sigma(x.covariance()),
                        m_is_sigma_valid(true),
                        m_is_mu_valid(true)
                    {
                    }
                    GaussianCanonical(GaussianCanonical<X>&& that) :
                        m_precision(std::move(that.m_precision)),
                        m_information(std::move(that.m_information)),
                        m_is_sigma_valid(that.m_is_sigma_valid),
                        m_is_mu_valid(that.m_is_mu_valid),
                        m_sigma(std::move(that.m_sigma)),
                        m_mu(std::move(that.m_mu))
                    {
                    }

                    GaussianCanonical<X>& operator=(GaussianCanonical<X>&& that)
                    {
                        this->m_precision = std::move(that.m_precision);
                        this->m_information = std::move(that.m_information);
                        this->m_is_sigma_valid = that.m_is_sigma_valid;
                        this->m_is_mu_valid = that.m_is_mu_valid;
                        this->m_sigma = std::move(that.m_sigma);
                        this->m_mu = std::move(that.m_mu);
                        return *this;
                    }

                    virtual ~GaussianCanonical() override
                    {
                    }

                    const Vector& information() const
                    {
                        return m_information;
                    }
                    const Matrix& precision() const
                    {
                        return m_precision;
                    }
                    const Matrix& sigma() const
                    {
                        return inversePrecision();
                    }
                    const Vector& mu() const
                    {
                        return getMu();
                    }

                    const Matrix& inversePrecision() const
                    {
                        if (!m_is_sigma_valid)
                        {
                            m_sigma = precision().inverse();
                            m_is_sigma_valid = true;
                        }
                        return m_sigma;
                    }

                    const X& getMu() const
                    {
                        if (!m_is_mu_valid)
                        {
                            m_mu.set(sigma() * information());
                            m_is_mu_valid = true;
                        }
                        return m_mu;
                    }

                    double probabilityOf(const X&) const override
                    {
                        throw std::runtime_error("GaussianCanonical::ProbabilityOf not implemented.");
                    }

                    X sample(std::default_random_engine*) const override
                    {
                        throw std::runtime_error("GaussianCanonical::Sample not implemented.");
                    }

                private:
                    Vector m_information;  // Omega
                    Matrix m_precision;  // xi
                    mutable bool m_is_sigma_valid = false;
                    mutable bool m_is_mu_valid = false;
                    mutable Matrix m_sigma;
                    mutable X m_mu;

                };

                template<typename X>
                static std::ostream& operator<<(std::ostream& os, const GaussianCanonical<X>& x)
                {
                    os << "omega=" << x.information() << ", xi=" << x.precision();
                    return os;
                };
            }
        }
    }
}

#endif // GAUSSIAN_CANONICAL_HPP
