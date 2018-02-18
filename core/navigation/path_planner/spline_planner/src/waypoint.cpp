#include <spline_planner/waypoint.hpp>

namespace rip
{
    namespace navigation
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

        void to_json(nlohmann::json& j, const Waypoint& w)
        {
            j["position"] = w.m_position;
            j["tangent"] = w.m_tangent;
            j["curvature"] = w.m_curvature;
        }

        void from_json(const nlohmann::json& j, Waypoint& w)
        {
            w = Waypoint(j["position"].get<Point>(), j["tangent"].get<Point>(), j["curvature"].get<Point>());
        }

    } // namespace navigation
} // namespace rip
