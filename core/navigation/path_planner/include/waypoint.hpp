#ifndef WAYPOINT_HPP
#define WAYPOINT_HPP

#include "point.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathplanner
        {
            using Point = geometry::Point;
            using Distance = units::Distance;
            using Angle = units::Angle;

            class Waypoint
            {
            public:
                Waypoint(const Distance& x, const Distance& z, const Angle& heading);

                Waypoint(const Point& p, const Angle& heading);

                Waypoint(const Point& position, const Point& tangent);

                Waypoint(const Point& position, const Point& tangent, const Point& curvature);

                Point position() const;

                Point tangent() const;

                Angle heading() const;

                Point curvature() const;

            private:
                Point m_position;
                Point m_tangent;
                Point m_curvature;
            }; // class Waypoint
        } // namespace pathplanner
    } // namespace navigation
} // namespace rip

#endif // WAYPOINT_HPP
