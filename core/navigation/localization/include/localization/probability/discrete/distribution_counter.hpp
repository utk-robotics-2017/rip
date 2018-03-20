#ifndef DISTRIBUTION_COUNTER_HPP
#define DISTRIBUTION_COUNTER_HPP

#include <map>
#include <random>

#include "discrete_distribution.hpp"

namespace rip
{
    namespace navigation
    {
        namespace localization
        {
            namespace probability
            {
                template <typename X>
                class DistributionCounter : public DiscreteDistribution<X>
                {
                public:
                    DistributionCounter() {}

                    virtual ~DistributionCounter() {}

                    double probabilityOf(const X& x) const override
                    {
                        double result = 0.0;
                        auto key_value = m_map.find(x);
                        if (key_value != m_map.end())
                        {
                            double n = key_value->second;
                            result = n / m_total;
                        }
                        return result;
                    }

                    X sample(std::default_random_engine* generator) const override
                    {
                        std::uniform_real_distribution<double> random(0, m_total);
                        double select = random(*generator);
                        return this->selectFromMap(m_map, select);
                    }

                    void addObservation(X x)
                    {
                        int n = 0;
                        auto key_value = m_map.find(x);
                        if (key_value != m_map.end())
                        {
                            n = key_value->second;
                        }
                        m_map[x] = n + 1;
                        m_total ++;
                    }

                private:
                    int m_total = 0;
                    std::map<X, int> m_map;
                };
            }
        }
    }
}

#endif // DISTRIBUTION_COUNTER_HPP
