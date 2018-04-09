#ifndef EPSILON_HPP
#define EPSILON_HPP

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            extern double k_epsilon;

            template <typename T>
            bool epsilonEquals(const T& a, const T& b, const T& epsilon)
            {
                return (a - epsilon <= b) && (a + epsilon >= b);
            }
        }
    }
}

#endif //EPSILON_HPP
