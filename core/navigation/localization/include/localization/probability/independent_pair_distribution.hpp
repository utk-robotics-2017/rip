#ifndef INDEPENDENT_PAIR_DISTRIBUTION_HPP
#define INDEPENDENT_PAIR_DISTRIBUTION_HPP

#include <memory>
#include <utility>

#include "random_distribution.hpp"

namespace rip
{
    namespace navigation
    {
        namespace localization
        {
            namespace probability
            {
                template<typename A, typename B>
                class IndependentPairDistribution : public RandomDistribution<std::pair<A, B>>
                {
                public:
                    typedef std::pair<A, B> AB;

                    IndependentPairDistribution(std::shared_ptr< RandomDistribution <A> > a, std::shared_ptr< RandomDistribution <B> > b)
                    : m_a(a)
                    , m_b(b)
                    {

                    }

                    virtual ~IndependentPairDistribution()
                    {
                    }

                    double probabilityOf(const AB& t) const override
                    {
                        return m_a->probabilityOf(t.first) * m_b->probabilityOf(t.second);
                    }

                    AB sample(std::default_random_engine* generator) const override
                    {
                        return AB(m_a->sample(generator), m_b->sample(generator));
                    }

                private:
                    std::shared_ptr< RandomDistribution <A> > m_a;
                    std::shared_ptr< RandomDistribution <B> > m_b;

                };

            }
        }
    }
}
#endif // INDEPENDENT_PAIR_DISTRIBUTION_HPP