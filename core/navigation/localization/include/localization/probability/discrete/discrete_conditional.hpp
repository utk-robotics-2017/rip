#ifndef DISCRETE_CONDITIONAL_HPP
#define DISCRETE_CONDITIONAL_HPP

#include <memory>
#include <vector>

#include "../random_conditional.hpp"

namespace rip
{
    namespace navigation
    {
        namespace localization
        {
            namespace probability
            {

                template <typename X, typename Y>
                class DiscreteConditional : public RandomConditional<X, Y>
                {
                public:
                    DiscreteConditional() {}

                    virtual ~DiscreteConditional() override {}

                    virtual std::vector<X> domain() const = 0;

                    virtual std::vector<Y> range() const = 0;
                };

            }
        }
    }
}

#endif // DISCRETE_CONDITIONAL_HPP
