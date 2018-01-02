#ifndef WAYPOINT_HPP
#define WAYPOINT_HPP

#include "point.hpp"

namespace rip
{
    namespace navigation
    {

        using Point = geometry::Point;
        using Distance = units::Distance;
        using Angle = units::Angle;

        class Waypoint
        {
        public:
            Waypoint() = default;

            Waypoint(const Distance& x, const Distance& z, const Angle& heading);

            Waypoint(const Point& p, const Angle& heading);

            Waypoint(const Point& position, const Point& tangent);

            Waypoint(const Point& position, const Point& tangent, const Point& curvature);

            Point position() const;

            Point tangent() const;

            Angle heading() const;

            Point curvature() const;

            friend void to_json(nlohmann::json& j, const Waypoint& w);
            friend void from_json(const nlohmann::json& j, Waypoint& w);

        private:
            Point m_position;
            Point m_tangent;
            Point m_curvature;
        }; // class Waypoint

        void to_json(nlohmann::json& j, const Waypoint& w);
        void from_json(const nlohmann::json& j, Waypoint& w);

    } // namespace navigation
} // namespace rip

#endif // WAYPOINT_HPP
