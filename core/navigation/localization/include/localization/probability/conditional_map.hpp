#ifndef CONDITIONAL_MAP_HPP
#define CONDITIONAL_MAP_HPP

#include <map>
#include <memory>

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
                class ConditionalMap : public RandomConditional<X, Y>
                {
                public:
                    ConditionalMap()
                    {}

                    virtual ~ConditionalMap()
                    {}

                    void set(Y y, std::shared_ptr<RandomDistribution<X>> x)
                    {
                        m_map[y] = x;
                    }

                    double conditionalProbabilityOf(const X& x, const Y& y) const override
                    {
                        double result = 0.0;
                        auto key_value = m_map.find(y);
                        if (key_value != m_map.end())
                        {
                            result = key_value->second->probabilityOf(x);
                        }
                        return result;
                    }

                private:
                    std::map<Y, std::shared_ptr<RandomDistribution<X>>> m_map;
                };
            }
        }
    }
}

#endif // CONDITIONAL_MAP_HPP
