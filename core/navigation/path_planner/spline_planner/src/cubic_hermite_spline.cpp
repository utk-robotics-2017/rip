#include <spline_planner/cubic_hermite_spline.hpp>
#include <spline_planner/spline_utils.hpp>

namespace rip
{
    namespace navigation
    {

        CubicHermiteSpline::CubicHermiteSpline(const std::vector<Waypoint>& waypoints, double alpha)
            : m_waypoints(waypoints)
        {
            assert(m_waypoints.size() >= 2);
            m_knots = splineutils::computeXValuesWithInnerPadding(m_waypoints, alpha, 0);
            m_arc_lengths  = std::vector<Distance>(m_waypoints.size() - 1, -1);
        }

        Point CubicHermiteSpline::position(const Distance& x)
        {
            int index = segmentForX(x);

            Distance x_diff = segmentArcLength(index); // Length of the segment we are currently on
            double t = ((x - lengthUntilSegment(index)) / x_diff)(); // Percentage of the way down the spline we are

            double one_minus_t = 1.0 - t;
            double basis00 = (1 + 2 * t) * one_minus_t* one_minus_t;
            double basis10 = t * one_minus_t* one_minus_t;
            double basis11 = t * t * - one_minus_t;
            double basis01 = t * t * (3 - 2 * t);

            return basis00 * m_waypoints[index].position() +
                   basis10 * x_diff() * m_waypoints[index].tangent() +
                   basis11 * x_diff() * m_waypoints[index + 1].tangent() +
                   basis01 * m_waypoints[index + 1].position();
        }

        Point CubicHermiteSpline::tangent(const Distance& x)
        {
            int index = segmentForX(x);

            Distance x_diff = segmentArcLength(index); // Length of the segment we are currently on
            double t = ((x - lengthUntilSegment(index)) / x_diff)(); // Percentage of the way down the spline we are

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

        Point CubicHermiteSpline::curvature(const Distance& x)
        {
            int index = segmentForX(x);

            Distance x_diff = segmentArcLength(index); // Length of the segment we are currently on
            double t = ((x - lengthUntilSegment(index)) / x_diff)(); // Percentage of the way down the spline we are

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

        Point CubicHermiteSpline::wiggle(const Distance& x)
        {
            int index = segmentForX(x);

            Distance x_diff = m_knots[index + 1] - m_knots[index]; // Length of the segment we are currently on

            return (
                       12 * (m_waypoints[index].position() - m_waypoints[index + 1].position()) +
                       6 * x_diff() * (m_waypoints[index].tangent() + m_waypoints[index + 1].tangent())
                   ) / (x_diff * x_diff * x_diff)();
        }

        Distance CubicHermiteSpline::segmentArcLength(int index)
        {
            if (m_arc_lengths[index]() > -1)
            {
                return m_arc_lengths[index];
            }

            Distance sum = 0;
            Point p0 = m_waypoints[index].position();
            for (double t = 0.0001; t <= 1.0; t += 0.0001)
            {
                double one_minus_t = 1.0 - t;
                double basis00 = (1 + 2 * t) * one_minus_t* one_minus_t;
                double basis10 = t * one_minus_t* one_minus_t;
                double basis11 = t * t * - one_minus_t;
                double basis01 = t * t * (3 - 2 * t);

                Point p = basis00 * m_waypoints[index].position() +
                          basis10 * m_waypoints[index].tangent() +
                          basis11 * m_waypoints[index + 1].tangent() +
                          basis01 * m_waypoints[index + 1].position();

                sum += p.distance(p0);
                p0 = p;
            }
            m_arc_lengths[index] = sum;
            return sum;
        }

        size_t CubicHermiteSpline::numSegments() const
        {
            return m_waypoints.size() - 1;
        }

        size_t CubicHermiteSpline::segmentForX(const Distance& x)
        {
            Distance cummulative = 0;
            int i = 0;
            for (int end = m_knots.size() - 1; i < end; i++)
            {
                cummulative += segmentArcLength(i);
                if (cummulative >= x)
                {
                    return i;
                }
            }
            return i;
        }

        Distance CubicHermiteSpline::lengthUntilSegment(int index)
        {
            Distance cummulative = 0;
            for (int i = 0; i < index; i++)
            {
                cummulative += segmentArcLength(i);
            }
            return cummulative;
        }
    }
}
