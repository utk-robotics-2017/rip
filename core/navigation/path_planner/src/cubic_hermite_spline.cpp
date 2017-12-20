#include "cubic_hermite_spline.hpp"
#include "spline_utils.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathplanner
        {
            CubicHermiteSpline::CubicHermiteSpline(const std::vector<Waypoint>& waypoints, double alpha)
            {
                assert(m_waypoints.size() >= 2);
                m_knots = splineutils::computeXValuesWithInnerPadding(m_waypoints, alpha, 0);
            }

            Point CubicHermiteSpline::position(const Distance& x) const
            {
                int index = segmentForX(x);

                Distance x_diff = m_knots[index + 1] - m_knots[index]; // Length of the segment we are currently on
                double t = ((x - m_knots[index]) / x_diff)(); // Percentage of the way down the spline we are

                double one_minus_t = 1.0 - t;
                double basis00 = (1 + 2 * t) * one_minus_t * one_minus_t;
                double basis10 = t * one_minus_t * one_minus_t;
                double basis11 = t * t * - one_minus_t;
                double basis01 = t * t * (3 - 2 * t);

                return basis00 * m_waypoints[index].position() +
                       basis10 * x_diff() * m_waypoints[index].tangent() +
                       basis11 * x_diff() * m_waypoints[index + 1].tangent() +
                       basis01 * m_waypoints[index + 1].position();
            }

            Point CubicHermiteSpline::tangent(const Distance& x) const
            {
                int index = segmentForX(x);

                Distance x_diff = m_knots[index + 1] - m_knots[index]; // Length of the segment we are currently on
                double t = ((x - m_knots[index]) / x_diff)(); // Percentage of the way down the spline we are

                double one_minus_t = 1.0 - t;
                double d_basis00 = 6 * t * (t - 1);
                double d_basis10 = (1 - 3 * t) * one_minus_t;
                double d_basis11 = t * (3 * t - 2);
                double d_basis01 = - d_basis00;

                return (d_basis00 * m_waypoints[index].position() +
                        d_basis10 * x_diff() * m_waypoints[index].tangent() +
                        d_basis11 * x_diff() * m_waypoints[index + 1].tangent() +
                        d_basis01 * m_waypoints[index + 1].position()) / x_diff();
            }

            Point CubicHermiteSpline::curvature(const Distance& x) const
            {
                int index = segmentForX(x);

                Distance x_diff = m_knots[index + 1] - m_knots[index]; // Length of the segment we are currently on
                double t = ((x - m_knots[index]) / x_diff)(); // Percentage of the way down the spline we are

                double d2_basis00 = 6 * (2 * t - 1);
                double d2_basis10 = 2 * (3 * t - 2);

                double d2_basis11 = 2 * (3 * t - 1);
                double d2_basis01 = -d2_basis00;

                return (
                           d2_basis00 * m_waypoints[index].position() +
                           d2_basis10 * x_diff() * m_waypoints[index].tangent() +

                           d2_basis11 * x_diff() * m_waypoints[index + 1].tangent() +
                           d2_basis01 * m_waypoints[index + 1].position()
                       ) / (x_diff * x_diff)();
            }

            Point CubicHermiteSpline::wiggle(const Distance& x) const
            {
                int index = segmentForX(x);

                Distance x_diff = m_knots[index + 1] - m_knots[index]; // Length of the segment we are currently on

                return (
                           12 * (m_waypoints[index].position() - m_waypoints[index + 1].position()) +
                           6 * x_diff() * (m_waypoints[index].tangent() + m_waypoints[index + 1].tangent())
                        ) / (x_diff * x_diff * x_diff)();
            }

            Distance CubicHermiteSpline::segmentArcLength(int i) const
            {
                assert(i < m_knots.size() - 1);
                return m_knots[i + 1] - m_knots[i];
            }

            size_t CubicHermiteSpline::numSegments() const
            {
                return m_waypoints.size() - 1;
            }

            size_t CubicHermiteSpline::segmentForX(const Distance& x) const
            {
                size_t segment_index = splineutils::getIndexForX(m_knots, x);
                if (segment_index >= numSegments())
                {
                    return numSegments() - 1;
                }
                else
                {
                    return segment_index;
                }
            }
        }
    }
}
