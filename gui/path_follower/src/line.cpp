#include "path_follower_gui/line.hpp"
#include "path_follower/epsilon.hpp"
namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            namespace gui
            {
                Line::Line(const Waypoint& a, const Waypoint& b)
                    : m_a(a)
                    , m_b(b)
                {
                    m_slope = m_a.position() - m_b.position();
                    m_start = m_a.position().translateBy(m_slope.scale(m_a.radius()() / m_slope.norm()()).inverse());
                    m_end = m_b.position().translateBy(m_slope.scale(m_b.radius()() / m_slope.norm()()));
                }

                void Line::draw(QPainter& painter)
                {
                    painter.drawLine(m_start.x()(), m_start.y()(), m_end.x()(), m_end.y()());
                }

                Translation2d Line::intersect(const Translation2d& a, const Translation2d& b, const Translation2d& c, const Translation2d& d)
                {
                    auto i = ((a.x() - b.x()) * (c.y() - d.y()) - (a.y() - b.y()) * (c.x() - d.x()));
                    i = (units::abs(i) < k_epsilon) ? k_epsilon : i;
                    units::Distance x = (Translation2d::cross(a, b) * (c.x() - d.x()) - Translation2d::cross(c, d) * (a.x() - b.x())) / i();
                    units::Distance y = (Translation2d::cross(a, b) * (c.y() - d.y()) - Translation2d::cross(c, d) * (a.y() - b.y())) / i();
                    return Translation2d(x, y);
                }

                Translation2d Line::slope() const
                {
                    return m_slope;
                }

                Translation2d Line::start() const
                {
                    return m_start;
                }

                Translation2d Line::end() const
                {
                    return m_end;
                }
            }
        }
    }
}
