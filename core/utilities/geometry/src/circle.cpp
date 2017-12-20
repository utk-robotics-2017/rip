#include "circle.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace rip
{
    namespace geometry
    {
        Circle::Circle(const Point& center, const Distance& radius)
            : m_center(center)
            , m_radius(radius)
        {}

        void Circle::center(const Point& center)
        {
            m_center = center;
        }

        Point Circle::center() const
        {
            return m_center;
        }

        void Circle::radius(const Distance& radius)
        {
            m_radius = radius;
        }

        Distance Circle::radius() const
        {
            return m_radius;
        }

        bool Circle::inside(const Point& point) const
        {
            return m_center.distance(point) <= m_radius;
        }

        Distance Circle::perimeter() const
        {
            return 2 * m_radius * M_PI;
        }

        Area Circle::area() const
        {
            return m_radius * m_radius * M_PI;
        }

        Rectangle Circle::boundingBox() const
        {
            return Rectangle(m_center.x() - m_radius, m_center.x() + m_radius, m_center.y() - m_radius, m_center.y() + m_radius);
        }

        void from_json(const nlohmann::json& j, Circle& c)
        {
            c.m_center = j["center"];
            c.m_radius = j["radius"];
        }

        void to_json(nlohmann::json& j, const Circle& c)
        {
            j = nlohmann::json
            {
                {"center", c.m_center},
                {"radius", c.m_radius}
            };
        }
    }
}