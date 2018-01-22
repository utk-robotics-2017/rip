#ifndef PENALTIES_HPP
#define PENALTIES_HPP

namespace rip
{
    namespace navigation
    {
        /**
         * Linear penalty function for bounding @p var to the interval \f$ -a < var < a \f$
         */
        template<typename T>
        inline T penaltyBoundToInterval(const T& var, const T& a, const T& epsilon)
        {
            if (var < -a + epsilon)
            {
                return -var - (a - epsilon);
            }
            if (var <= a - epsilon)
            {
                return 0;
            }
            else
            {
                return var - (a - epsilon);
            }
        }


        /**
         * Linear penalty function for bounding @p var to the interval \f$ a < var < b \f$
         */
        template<typename T>
        inline T penaltyBoundToInterval(const T& var, const T& a, const T& b, const T& epsilon)
        {
            if (var < a + epsilon)
            {
                return (-var + (a + epsilon));
            }
            if (var <= b - epsilon)
            {
                return 0.;
            }
            else
            {
                return (var - (b - epsilon));
            }
        }


        /**
         * Linear penalty function for bounding @p var from below: \f$ a < var \f$
         */
        template<typename T>
        inline T penaltyBoundFromBelow(const T& var, const T& a, const T& epsilon)
        {
            if (var >= a + epsilon)
            {
                return 0.;
            }
            else
            {
                return (-var + (a + epsilon));
            }
        }
    }
}

#endif // PENALTIES_HPP
