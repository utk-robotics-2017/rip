#include "path_follower/waypoint.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            Waypoint::Waypoint(const units::Distance& x, const units::Distance& y, const units::Distance& radius, const units::Velocity& speed)
                : m_position(x, y)
                , m_radius(radius)
                , m_speed(speed)
            {
            }

            Waypoint::Waypoint(const units::Distance& x, const units::Distance& y, const units::Distance& radius, const units::Velocity& speed, const std::string& marker)
                : m_position(x, y)
                , m_radius(radius)
                , m_speed(speed)
                , m_marker(marker)
            {
            }

            Waypoint::Waypoint(const Translation2d& position, const units::Distance& radius, const units::Velocity& speed)
                : m_position(position)
                , m_radius(radius)
                , m_speed(speed)
            {
            }

            Waypoint::Waypoint(const Waypoint& other)
                : m_position(other.m_position)
                , m_radius(other.m_radius)
                , m_speed(other.m_speed)
            {
            }

            Translation2d Waypoint::position() const
            {
                return m_position;
            }

            units::Distance Waypoint::x() const
            {
                return m_position.x();
            }

            units::Distance Waypoint::y() const
            {
                return m_position.y();
            }

            units::Distance Waypoint::radius() const
            {
                return m_radius;
            }

            units::Velocity Waypoint::speed() const
            {
                return m_speed;
            }

            std::string Waypoint::marker() const
            {
                return m_marker;
            }

        }
    }
}