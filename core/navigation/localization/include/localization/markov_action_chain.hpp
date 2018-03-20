#ifndef MARKOV_ACTION_CHAIN_HPP
#define MARKOV_ACTION_CHAIN_HPP

#include "localization/probability/discrete/discrete_conditional.hpp"
#include "localization/probability/discrete/distribution_value_map.hpp"

#include <map>

namespace rip
{
    namespace navigation
    {
        namespace localization
        {
            template <typename X, typename U>
            class MarkovActionChain : pulbic probability::DiscreteConditional<X, std::pair<X, U>>
            {
            public:
                typedef std::pair<X, U> XU;

                MarkovActionChain()
                {
                }

                virtual ~MarkovActionChain() override
                {
                }

                std::vector<X> domain() const override
                {
                    std::vector<X> result;
                    result.reserve(m_x.size());
                    for (auto& x_x : m_x)
                    {
                        result.push_back(x_x.first);
                    }
                    return result;
                }

                std::vector<XU> range() const override
                {
                    std::vector<XU> result;
                    result.reserve(xu_.size());
                    for (auto& xu_xu : m_xu)
                    {
                        result.push_back(xu_xu.first);
                    }
                    return result;
                }

                double conditionalProbabilityOf(const X& x, const XU& xu) const override
                {
                    const X& start = xu.first;
                    const U& u = xu.second;
                    auto start_action_map = m_p.find(start);
                    if (start_action_map == m_p.end())
                    {
                        return 0.0;
                    }
                    auto action_end_map = start_action_map->second.find(u);
                    if (action_end_map == start_action_map->second.end())
                    {
                        return 0.0;
                    }
                    auto end_value = action_end_map->second.find(x);
                    if (end_value == action_end_map->second.end())
                    {
                        return 0.0;
                    }
                    return end_value->second;
                }

                void set(X end, U action, X start, double p)
                {
                    auto start_action_map = m_p.find(start);
                    if (start_action_map == m_p.end())
                    {
                        start_action_map = m_p.emplace(start, std::map<U, std::map<X, double>>()).first;
                    }
                    auto action_end_map = start_action_map->second.find(action);
                    if (action_end_map == start_action_map->second.end())
                    {
                        auto emplace_result = start_action_map->second.emplace(action, std::map<X, double>());
                        action_end_map = emplace_result.first;
                    }
                    action_end_map->second[end] = p;
                    m_x[end] = end;
                    m_u[action] = action;
                    m_xu[XU(start, action)] = XU(start, action);
                }

                double sumCondition(const XU& start_action) const
                {
                    double sum = 0.0;
                    const X& start = start_action.first;
                    const U& action = start_action.second;
                    auto start_action_map = m_p.find(start);
                    if (start_action_map != m_p.end())
                    {
                        auto action_end_map = start_action_map->second.find(action);
                        if (action_end_map != start_action_map->second.end())
                        {
                            for (auto& end_value : action_end_map->second)
                            {
                                sum += end_value.second;
                            }
                        }
                    }
                    return sum;
                }

                X sampleCondition(const XU& start_action, std::default_random_engine* generator) const
                {
                    double sum = sumLikelihood(start_action);
                    if (sum == 0.0)
                    {
                        throw std::runtime_error("Conditional distribution is not conditioned on start_action. Cannot sample.");
                    }
                    std::uniform_real_distribution<double> random(0, sum);
                    return sampleCondition(start_action, random(*generator));
                }

                X sampleCondition(const XU& start_action, double p) const
                {
                    const X& start = start_action.first;
                    const U& action = start_action.second;
                    auto start_action_map = m_p.find(start);
                    if (start_action_map == m_p.end())
                    {
                        throw std::runtime_error("MarkovActionChain does not have start state. Cannot sample.");
                    }
                    auto action_end_map = start_action_map->second.find(action);
                    if (action_end_map == start_action_map->second.end())
                    {
                        throw std::runtime_error("MarkovActionChain does not have action from start state. Cannot sample.");
                    }
                    for (auto& end_value : action_end_map->second)
                    {
                        p -= end_value.second;
                        if (p <= 0)
                        {
                            return end_value.first;
                        }
                    }
                    throw std::runtime_error("End state for specified <start, action> is badly specified. Cannot sample.");
                }

                std::shared_ptr<probabilityDistributionValueMap<X>> marginalize(const probability::RandomDistribution<XU>& start_action)
                {
                    auto result = std::make_shared <probability::DistributionValueMap<X >> ();
                    for (auto& level0 : m_p)
                    {
                        X start = level0.first;
                        for (auto& level1 : level0.second)
                        {
                            U action = level1.first;
                            for (auto& level2 : level1.second)
                            {
                                X end = level2.first;
                                double p0 = level2.second;
                                XU xu(start, action);
                                double p1 = start_action.probabilityOf(xu);
                                double current_end_p = result->probabilityOf(end);
                                result->set(end, p0 * p1 + current_end_p);
                            }
                        }
                    }
                    result->normalize();
                    return result;
                }

            private:
                std::map<X, std::map<U, std::map<X, double>>> m_p;
                std::map<X, X> m_x;
                std::map<U, U> m_u;
                std::map<XU, XU> m_xu;
            };
        }
    }
}

#endif // MARKOV_ACTION_CHAIN_HPP
