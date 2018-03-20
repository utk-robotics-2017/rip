#ifndef BAYES_DISTRIBUTION_HPP
#define BAYES_DISTRIBUTION_HPP

#include <memory>
#include <vector>

#include "random_distribution.hpp"
#include "random_conditional.hpp"

namespace rip
{
    namespace navigation
    {
        namespace localization
        {
            namespace probability
            {
                template <typename X, typename Y>
                class BayesDistribution : public RandomDistribution<X>,
                    public std::enable_shared_from_this<BayesDistribution<X, Y>>
                {
                public:
                    BayesDistribution(std::shared_ptr<const RandomDistribution<X>> prior,
                                      std::shared_ptr<const RandomConditional<Y, X>> sensor_model,
                                      Y observation)
                        : m_prior(prior)
                        , m_sensor_model(sensor_model)
                        , m_data(std::move(observation))
                    {}

                    virtual ~BayesDistribution() {}

                    double probabilityOf(const X& x) const override
                    {
                        double pyx = m_sensor_model->conditionalProbabilityOf(m_data, x);
                        double px = m_prior->probabilityOf(x);
                        double py = m_normalizer;
                        return pyx * px / py;
                    }

                    X sample(std::default_random_engine*) const override
                    {
                        throw std::runtime_error("Not Implemented");
                    }

                    std::shared_ptr<BayesDistribution<X, Y>> chainObservation(Y y)
                    {
                        return std::make_shared< BayesDistribution<X, Y> >(this->shared_from_this(), m_sensor_model, std::move(y));
                    }

                    double estimateNormalizer(const std::vector<X>& domain, double dx)
                    {
                        double sum = 0.0;
                        for (const X& x : domain)
                        {
                            double pz_given_x = m_sensor_model->conditionalProbabilityOf(m_data, x);
                            double px = m_prior->probabilityOf(x);
                            sum += pz_given_x * px;
                        }
                        m_normalizer = sum * dx;
                        return m_normalizer;
                    }

                    const RandomConditional<Y, X>& sensorModel() const
                    {
                        return *m_sensor_model;
                    }

                    const Y& data() const
                    {
                        return m_data;
                    }

                    double& normalizer()
                    {
                        return m_normalizer;
                    }

                    double normalizer() const
                    {
                        return m_normalizer;
                    }

                private:
                    std::shared_ptr<const RandomDistribution<X>> m_prior;
                    std::shared_ptr<const RandomConditional<Y, X>> m_sensor_model;
                    Y m_data;
                    double m_normalizer = 1.0;
                };
            }
        }
    }
}

#endif // BAYES_DISTRIBUTION_HPP
