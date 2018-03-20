#ifndef RANDOM_CONDITIONAL_HPP
#define RANDOM_CONDITIONAL_HPP

#include <memory>

namespace rip
{
    namespace navigation
    {
        namespace localization
        {
            namespace probability
            {
                template <typename X, typename Y>
                class RandomConditional
                {
                public:
                    RandomConditional() {}

                    virtual ~RandomConditional() {}

                    virtual double conditionalProbabilityOf(const X& x, const Y& y) const = 0;
                };
            }
        }
    }
}

#endif // RANDOM_CONDITIONAL_HPP
