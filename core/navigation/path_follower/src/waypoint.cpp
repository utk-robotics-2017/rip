#include "path_follower/waypoint.hpp"

#include <fmt/format.h>

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

            void Waypoint::setX(const units::Distance& x)
            {
                m_position.setX(x);
            }

            void Waypoint::setY(const units::Distance& y)
            {
                m_position.setY(y);
            }

            void Waypoint::setRadius(const units::Distance& radius)
            {
                m_radius = radius;
            }

            void Waypoint::setSpeed(const units::Velocity& speed)
            {
                m_speed = speed;
            }

            std::string Waypoint::toString() const
            {
                return fmt::format("(x: {} in, y: {} in, radius: {} in, speed: {} in/s)", m_position.x().to(units::in), m_position.y().to(units::in), m_radius.to(units::in), m_speed.to(units::in/units::s));
            }

            void to_json(nlohmann::json& j, const Waypoint& w)
            {
                j = {
                    {"x", w.x()},
                    {"y", w.y()},
                    {"radius", w.radius()},
                    {"speed", w.speed()}
                };
            }

            void from_json(const nlohmann::json& j, Waypoint& w)
            {
                w.m_position.setX(j["x"]);
                w.m_position.setY(j["y"]);
                w.m_radius = j["radius"];
                w.m_speed = j["speed"];
            }

            std::ostream& operator<<(std::ostream& os, const Waypoint& w)
            {
                os << w.toString() << std::endl;
                return os;
            }

        }
    }
}
