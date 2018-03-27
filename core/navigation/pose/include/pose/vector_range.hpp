#ifndef VECTOR_RANGE_HPP
#define VECTOR_RANGE_HPP

#include <cmath>
#include <random>
#include <utility>

namespace rip
{
    namespace navigation
    {
        namespace pose
        {
            template<typename X>
            class VectorRange
            {
            public:
                VectorRange(X min, X max) : m_min(std::move(min)), m_max(std::move(max))
                {
                }

                virtual ~VectorRange()
                {
                }

                double area() const
                {
                    Vector dx = m_max - m_min;
                    double result = 1.0;
                    for(unsigned int i = 0; i < dx.order(); i ++)
                    {
                        result *= dx[i];
                    }
                    return fabs(result);
                }

                bool contains(const X& x) const
                {
                    return x > m_min && x < m_max;
                }

                X sample(std::default_random_engine* generator) const
                {
                    X result;
                    for(unsigned int i = 0; i < result.order(); i ++)
                    {
                        std::uniform_real_distribution<double> random(m_min[i], m_max[i]);
                        result[i] = random(*generator);
                    }
                    return result;
                }

            private:
                X m_min;
                X m_max;
            };
        }
    }
}

#endif //VECTOR_RANGE_HPP
