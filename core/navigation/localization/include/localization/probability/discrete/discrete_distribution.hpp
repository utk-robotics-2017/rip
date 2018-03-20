#ifndef DISCRETE_DISTRIBUTION_HPP
#define DISCRETE_DISTRIBUTION_HPP

#include <map>

#include "../random_distribution.hpp"

namespace rip
{
    namespace navigation
    {
        namespace localization
        {
            namespace probability
            {
                template <typename X>
                class DiscreteDistribution : public RandomDistribution<X>
                {
                public:
                    DiscreteDistribution() {}

                    virtual ~DiscreteDistribution() {}

                protected:
                    template <typename NumberType>
                    static X selectFromMap(const std::map<X, NumberType>& map, double select)
                    {
                        if (map.empty())
                        {
                            throw std::runtime_error("Cannot sample from an empty distribution");
                        }
                        X result;
                        auto p = map.begin();
                        do
                        {
                            result = p->first;
                            select -= p->second;
                        }
                        while (++p != map.end() && select >= 0);
                        return result;
                    }
                };
            }
        }
    }
}

#endif // DISCRETE_DISTRIBUTION_HPP
