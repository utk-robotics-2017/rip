#ifndef DISTRIBUTION_VALUE_MAP_HPP
#define DISTRIBUTION_VALUE_MAP_HPP

#include <vector>
#include <random>
#include <map>
#include <memory>

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
                class DistributionValueMap : public DiscreteDistribution<X>
                {
                public:
                    DistributionValueMap() {}

                    DistributionValueMap(X x0, double p0, X x1, double p1)
                    {
                        set(x0, p0);
                        set(x1, p1);
                        normalize();
                    }

                    DistributionValueMap(const std::vector<X>& list)
                    {
                        for (auto& x : list)
                        {
                            set(x, 1.0);
                        }
                        normalize();
                    }

                    virtual ~DistributionValueMap() override {}

                    double probabilityOf(const X& x) const override
                    {
                        double result = 0.0;
                        auto it = m_map.find(x);
                        if (it != m_map.end())
                        {
                            result = it->second;
                        }
                        return result;
                    }

                    X sample(std::default_random_engine* generator) const override
                    {
                        std::uniform_real_distribution<double> random(0, 1);
                        double select = random(*generator);
                        return this->selectFromMap(m_map, select);
                    }

                    void set(X x, double p)
                    {
                        m_map[x] = p;
                    }

                    void normalize()
                    {
                        double sum = 0.0;
                        for (auto& pair : m_map)
                        {
                            sum += pair.second;
                        }
                        for (auto& pair : m_map)
                        {
                            pair.second /= sum;
                        }
                    }

                    double entropy() const
                    {
                        double result = 0.0;
                        for (auto& pair : m_map)
                        {
                            double p = pair.second;
                            if (p != 0)
                            {
                                result -= p * log2(p);
                            }
                        }
                        return result;
                    }

                    std::shared_ptr<DistributionValueMap<X>> product(const DistributionValueMap<X>& that) const
                    {
                        auto result = std::make_shared<DistributionValueMap<X>>();
                        for (auto& pair : m_map)
                        {
                            double lp = pair.second;
                            double rp = that.probabilityOf(pair.first);
                            double p = lp * rp;
                            if (p != 0)
                            {
                                result->set(pair.first, p);
                            }
                        }
                        result->normalize();
                        return result;
                    }

                private:
                    std::map<X, double> m_map;
                };
            }
        }
    }
}

#endif // DISTRIBUTION_VALUE_MAP_HPP
