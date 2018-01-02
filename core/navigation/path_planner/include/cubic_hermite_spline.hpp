#ifndef CUBIC_HERMITE_SPLINE_HPP
#define CUBIC_HERMITE_SPLINE_HPP

#include <vector>

#include "point.hpp"
#include "waypoint.hpp"
#include "spline.hpp"

namespace rip
{
    namespace navigation
    {
        class CubicHermiteSpline : public Spline
        {
        public:
            CubicHermiteSpline(const std::vector<Waypoint>& waypoints, double alpha = 0.0);

            /**
             * Returns the 2-dimensional position at \p x length down the spline
             */
            virtual Point position(const Distance& x) override;

            /**
             * Returns the 2-dimensional vector of the tangent (1st derivative)
             * of the spline at \p x length down the spline
             */
            virtual Point tangent(const Distance& x) override;

            /**
             * Returns the 2-dimensional vector of the curvature (2nd derivative)
             * of the spline at \p x length down the spline
             */
            virtual Point curvature(const Distance& x) override;

            /**
             * Returns the 2-dimensional vector of the wiggle (3rd derivative)
             * of the spline at \p x length down the spline
             */
            virtual Point wiggle(const Distance& x) override;

            /**
             * Returns the arc length of the (i+1)th segment
             */
            virtual Distance segmentArcLength(int i) override;

            /**
             * Returns the number of segments in the spline
             */
            virtual size_t numSegments() const override;

        protected:

            /**
             * Returns the index of the segment that \p x down the spline is on
             */
            virtual size_t segmentForX(const Distance& x) override;

            Distance lengthUntilSegment(int index);
        private:
            std::vector<double> m_knots;
            std::vector<Distance> m_arc_lengths;
            std::vector<Waypoint> m_waypoints;
        }; // class CubicHermiteSpline

    }
}

#endif // CUBIC_HERMITE_SPLINE_HPP
