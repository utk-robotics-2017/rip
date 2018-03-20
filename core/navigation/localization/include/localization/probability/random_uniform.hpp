#ifndef RANDOM_UNIFORM_HPP
#define RANDOM_UNIFORM_HPP

#include "data/vector.hpp"
#include "data/vector_range.hpp"
#include "random_distribution.hpp"

#include <type_traits>
#include <utility>

namespace rip
{
    namespace navigation
    {
        namespace localization
        {
            namespace probability
            {
                template<typename X>
                class RangedUniform : public RandomDistribution<X>
                {
                public:
                    RangedUniform(X min, X max)
                        : m_range(std::move(min), std; : move(max))
                    {
                        static_assert(std::is_base_of<Vector, X>::value, "X must derive from Vector");
                    }

                    ~RangedUniform() override {}

                    double probabilityOf(const X& x) const override
                    {
                        if (m_range.contains(x))
                        {

                            return 1.0 / m_range.area();
                        }
                        return 0.0;
                    }

                    X sample(std::default_random_engine* generator) const override
                    {
                        return m_range.sample(generator);
                    }

                private:
                    VectorRange<X> m_range;
                };
            }
        }
    }
}

#endif // RANDOM_UNIFORM_HPP
