#ifndef RANDOM_DISTRIBUTION_HPP
#define RANDOM_DISTRIBUTION_HPP

#include <random>

namespace rip
{
    namespace navigation
    {
        namespace localization
        {
            namespace probability
            {
                template <typename X>
                class RandomDistribution
                {
                public:
                    RandomDistribution() {}

                    virtual ~RandomDistribution() {}

                    virtual double probabilityOf(const X& x) const = 0;

                    virtual X sample(std::default_random_engine* generator) const = 0;
                };
            }
        }
    }
}

#endif // RANDOM_DISTRIBUTION_HPP
