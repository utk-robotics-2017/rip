#ifndef INTERPOLABLE_HPP
#define INTERPOLABLE_HPP

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            /**
             * Interpolable is an interface used by an Interpolating Tree as the Value type. Given two end points and an
             * interpolation parameter on [0, 1], it calculates a new Interpolable representing the interpolated value.
             *
             * @param <T>
             *            The Type of Interpolable
             * @see InterpolatingTreeMap
             */
            template <typename T>
            class Interpolable
            {
            public:
                /**
                 * Interpolates between this value and an other value according to a given parameter. If x is 0, the method should
                 * return this value. If x is 1, the method should return the other value. If 0 < x < 1, the return value should be
                 * interpolated proportionally between the two.
                 *
                 * @param other
                 *            The value of the upper bound
                 * @param x
                 *            The requested value. Should be between 0 and 1.
                 * @return Interpolable<T> The estimated average between the surrounding data
                 */
                virtual T interpolate(const T& other, double x) = 0;
            };
        }
    }
}

#endif //INTERPOLABLE_HPP
