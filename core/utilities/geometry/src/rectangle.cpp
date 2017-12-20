#include "rectangle.hpp"

namespace rip
{
    namespace geometry
    {
        Rectangle::Rectangle(const Point& top_left, const Point& bottom_right)
            : m_top_left(top_left)
            , m_bottom_right(bottom_right)
        {}

        Rectangle::Rectangle(const Distance& min_x, const Distance& max_x, const Distance& min_y, const Distance& max_y)
            : m_top_left(min_x, max_y)
            , m_bottom_right(max_x, min_y)
        {}

        bool Rectangle::inside(const Point& point) const
        {
            if (point.x() < m_top_left.x() || point.x() > m_bottom_right.x() || point.y() < m_bottom_right.y() || point.y() > m_top_left.y())
            {
                return false;
            }
            return true;
        }

        Distance Rectangle::perimeter() const
        {
            return 2.0 * height() + 2.0 * width();
        }

        Area Rectangle::area() const
        {
            return height() * width();
        }

        Distance Rectangle::width() const
        {
            return m_bottom_right.x() - m_top_left.x();
        }

        Distance Rectangle::height() const
        {
            return m_top_left.y() - m_bottom_right.y();
        }

        Point Rectangle::center() const
        {
            return Point(minX() + width() / 2.0, minY() + height() / 2.0);
        }

        Point Rectangle::topLeft() const
        {
            return m_top_left;
        }

        Point Rectangle::topRight() const
        {
            return Point(m_bottom_right.x(), m_top_left.y());
        }

        Point Rectangle::bottomLeft() const
        {
            return Point(m_top_left.x(), m_bottom_right.y());
        }

        Point Rectangle::bottomRight() const
        {
            return m_bottom_right;
        }

        Distance Rectangle::minX() const
        {
            return m_top_left.x();
        }

        Distance Rectangle::maxX() const
        {
            return m_bottom_right.x();
        }

        Distance Rectangle::minY() const
        {
            return m_bottom_right.y();
        }

        Distance Rectangle::maxY() const
        {
            return m_top_left.y();
        }
    }
}
