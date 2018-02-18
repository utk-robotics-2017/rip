#include "geometry/point.hpp"

namespace rip
{
    namespace geometry
    {
        Point::Point(const Distance& x, const Distance& y)
            : m_x(x)
            , m_y(y)
        {}

        Point::Point(const ClipperLib::IntPoint& point)
                : m_x(point.X)
                , m_y(point.Y)
            {
            }

        Distance Point::x() const
        {
            return m_x;
        }

        void Point::setX(const Distance& x)
        {
            m_x = x;
        }

        Distance Point::y() const
        {
            return m_y;
        }

        void Point::setY(const Distance& y)
        {
            m_y = y;
        }

        Distance Point::distance(const Point& rhs) const
        {
            return sqrt(units::pow<2>(m_x - rhs.m_x) + units::pow<2>(m_y - rhs.m_y));
        }

        Distance Point::magnitude() const
        {
            return sqrt(units::pow<2>(m_x) + units::pow<2>(m_y));
        }

        Point Point::normalize() const
        {
            Distance mag = magnitude();
            return Point(m_x / mag(), m_y / mag());
        }

        double Point::dot(const Point& rhs) const
        {
            return (m_x * rhs.m_x + m_y * rhs.m_y)();
        }

        double Point::cross(const Point& rhs) const
        {
            return (m_x * rhs.m_y - rhs.m_x * m_y)();
        }

        Point Point::operator -() const
        {
            return Point(-m_x, -m_y);
        }

        Point::operator ClipperLib::IntPoint() const
        {
            return ClipperLib::IntPoint(m_x(), m_y());
        }

        Point Point::operator +(const Point& rhs) const
        {
            return Point(m_x + rhs.m_x, m_y + rhs.m_y);
        }

        Point& Point::operator +=(const Point& rhs)
        {
            m_x += rhs.m_x;
            m_y += rhs.m_y;
            return *this;
        }

        Point Point::operator -(const Point& rhs) const
        {
            return Point(m_x - rhs.m_x, m_y - rhs.m_y);
        }

        Point& Point::operator -=(const Point& rhs)
        {
            m_x -= rhs.m_x;
            m_y -= rhs.m_y;
            return *this;
        }

        Point Point::operator *(double rhs) const
        {
            return Point(m_x * rhs, m_y * rhs);
        }

        Point& Point::operator *=(double rhs)
        {
            m_x *= rhs;
            m_y *= rhs;
            return *this;
        }

        Point Point::operator /(double rhs) const
        {
            return Point(m_x / rhs, m_y / rhs);
        }

        Point& Point::operator /=(double rhs)
        {
            m_x /= rhs;
            m_y /= rhs;
            return *this;
        }

        bool Point::operator ==(const Point& rhs) const
        {
            return m_x == rhs.m_x && m_y == rhs.m_y;
        }

        bool Point::operator !=(const Point& rhs) const
        {
            return m_x != rhs.m_x || m_y != rhs.m_y;
        }

        void from_json(const nlohmann::json& j, Point& point)
        {
            point.setX(j["x"]);
            point.setY(j["y"]);
        }

        void to_json(nlohmann::json& j, const Point& point)
        {
            j = nlohmann::json
            {
                {"x", point.x()},
                {"y", point.y()}
            };
        }

        Angle atan(const Point& p)
        {
            return Angle(atan2(p.y()(), p.x()()) * units::rad);
        }

        Point operator*(double lhs, const Point& rhs)
        {
            return rhs * lhs;
        }
    }
}
