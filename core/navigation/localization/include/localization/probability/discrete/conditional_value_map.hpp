#ifndef CONDITIONAL_VALUE_MAP_HPP
#define CONDITIONAL_VALUE_MAP_HPP

#include <map>
#include <memory>
#include <vector>
#include <iterator>
#include <algorithm>

#include "discrete_conditional.hpp"
#include "distribution_value_map.hpp"

namespace rip
{
    namespace navigation
    {
        namespace localization
        {
            namespace probability
            {
                template <typename X, typename Y>
                class ConditionalValueMap : public DiscreteConditional<X, Y>
                {
                public:
                    ConditionalValueMap() {}

                    virtual ~ConditionalValueMap() override {}

                    virtual std::vector<X> domain() const override
                    {
                        std::vector<X> keys;
                        std::transform(
                                m_domain.begin(),
                                m_domain.end(),
                                std::back_inserter(keys),
                                [](const std::pair<X, X>& pair){return pair.first;});
                        return keys;
                        /*
                        std::vector<X> result;
                        result.reserve(m_domain.size());
                        for (auto& pair : m_domain)
                        {
                            result.push_back(pair.first);
                        }
                        return result;
                         */
                    }

                    virtual std::vector<Y> range() const override
                    {
                        std::vector<Y> result;
                        result.reserve(m_map.size());
                        for (auto& pair : m_map)
                        {
                            result.push_back(pair.first);
                        }
                        return result;
                    }

                    double conditionalProbabilityOf(const X& x, const Y& y) const override
                    {
                        auto y_map = m_map.find(y);
                        if (y_map == m_map.end())
                        {
                            // todo: Does zero make sense here? Does an unknown imply zero change
                            return 0.0;
                        }

                        auto x_value = y_map->second.find(x);
                        if (x_value == y_map->second.end())
                        {
                            return 0.0;
                        }
                        return x_value->second;
                    }

                    void set(const X& x, const Y& y, double p)
                    {
                        auto value_map = m_map.find(y);
                        if (value_map == m_map.end())
                        {
                            value_map = m_map.emplace(y, std::map<X, double>()).first;
                        }
                        value_map->second[x] = p;
                        m_domain[x] = x;
                    }

                    std::shared_ptr<DistributionValueMap<Y>> likelihoodOf(const X& data) const
                    {
                        auto result = std::make_shared<DistributionValueMap<Y>>();
                        for (auto& y_map : m_map)
                        {
                            auto x_value = y_map.second.find(data);
                            if (x_value != y_map.second.end())
                            {
                                result->set(y_map.first, x_value->second);
                            }
                        }
                        return result;
                    }

                    std::shared_ptr<DistributionValueMap<Y>> bayesianInference(X data, const DistributionValueMap<Y>& prior) const
                    {
                        auto likelihood = likelihoodOf(data);
                        return likelihood->product(prior);
                    }

                    double sumLikelihood(const X& x) const
                    {
                        double sum = 0.0;
                        for (auto& y_map : m_map)
                        {
                            auto x_value = y_map.second.find(x);
                            if (x_value != y_map.second.end())
                            {
                                sum += x_value->second;
                            }
                        }
                        return sum;
                    }

                    Y sampleLikelihood(const X& x, double p) const
                    {
                        for (auto& y_map : m_map)
                        {
                            Y y = y_map.first;
                            auto x_value = y_map.second.find(x);
                            if (x_value != y_map.second.end())
                            {
                                p -= x_value->second;
                                if (p < 0)
                                {
                                    return y;
                                }
                            }
                        }
                        throw std::runtime_error("Conditional distribution is not conditioned on x. Cannot sample.");
                    }

                    template<typename Z>
                    std::shared_ptr<ConditionalValueMap<X, Z>> marginalize(const DiscreteConditional<Y, Z>& that)
                    {
                        auto result = std::make_shared<ConditionalValueMap<X, Z>>();
                        for (X& x : this->domain())
                        {
                            for (Z& z : that.range())
                            {
                                double p = 0.0;
                                for (Y& y : that.domain())
                                {
                                    double xy = this->conditionalProbabilityOf(x, y);
                                    double yz = that.conditionalProbabilityOf(y, z);
                                    p += xy * yz;
                                }
                                result->set(x, z, p);
                            }
                        }
                        return result;
                    }

                private:
                    std::map<Y, std::map<X, double>> m_map;
                    std::map<X, X> m_domain;
                };
            }
        }
    }
}

#endif // CONDITIONAL_VALUE_MAP_HPP
