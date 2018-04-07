#ifndef EPSILON_HPP
#define EPSILON_HPP

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            double k_epsilon = 1e-6;

            template <typename T>
            bool epsilonEquals(const T& a, const T& b, const T& epsilon)
            {
                return (a - epsilon <= b) && (a + epsilon >= b);
            }
        }
    }
}

#endif //EPSILON_HPP
