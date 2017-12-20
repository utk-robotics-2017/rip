#include "waypoint.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathplanner
        {

            Waypoint::Waypoint(const Distance& x, const Distance& y, const Angle& heading)
                : m_position(x, y)
                , m_tangent(cos(heading), sin(heading))
            {
                m_tangent.normalize();
            }

            Waypoint::Waypoint(const Point& p, const Angle& heading)
                : m_position(p)
                , m_tangent(cos(heading), sin(heading))
            {
                m_tangent.normalize();
            }

            Waypoint::Waypoint(const Point& position, const Point& tangent)
                : m_position(position)
                , m_tangent(tangent)
            {

            }

            Waypoint::Waypoint(const Point& position, const Point& tangent, const Point& curvature)
                : m_position(position)
                , m_tangent(tangent)
                , m_curvature(curvature)
            {

            }

            Point Waypoint::position() const
            {
                return m_position;
            }

            Point Waypoint::tangent() const
            {
                return m_tangent;
            }

            Angle Waypoint::heading() const
            {
                return atan(m_tangent);
            }

            Point Waypoint::curvature() const
            {
                return m_curvature;
            }
        } // namespace pathplanner
    } // namespace navigation
} // namespace rip