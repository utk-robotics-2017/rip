#ifndef SPLINE_HPP
#define SPLINE_HPP

#include "units.hpp"
#include <point.hpp>

namespace rip
{
    namespace navigation
    {
        namespace pathplanner
        {
            using Point = geometry::Point;
            using Distance = units::Distance;

            /**
             * Abstract base class for splines
             */
            class Spline
            {
            public:
                Spline() = default;

                /**
                 * Returns the 2-dimensional position at \p x length down the spline
                 */
                virtual Point position(const Distance& x) const = 0;

                /**
                 * Returns the 2-dimensional vector of the tangent (1st derivative)
                 * of the spline at \p x length down the spline
                 */
                virtual Point tangent(const Distance& x) const = 0;

                /**
                 * Returns the 2-dimensional vector of the curvature (2nd derivative)
                 * of the spline at \p x length down the spline
                 */
                virtual Point curvature(const Distance& x) const = 0;

                /**
                 * Returns the 2-dimensional vector of the wiggle (3rd derivative)
                 * of the spline at \p x length down the spline
                 */
                virtual Point wiggle(const Distance& x) const = 0;

                /**
                 * Returns the total arc length of the entire spline
                 * @return [description]
                 */
                Distance totalLength() const;

                /**
                 * Returns the arc length of a single segment in the spline
                 */
                virtual Distance segmentArcLength(int i) const = 0;

                /**
                 * Returns the number of segments in the spline
                 */
                virtual size_t numSegments() const = 0;

                /**
                 * Returns the index of the segment that \p x down the spline is on
                 */
                virtual size_t segmentForX(const Distance& x) const = 0;
            }; // class Spline
        } // namespace pathplanner
    } // namespace navigation
} // namespace rip

#endif // SPLINE_HPP
