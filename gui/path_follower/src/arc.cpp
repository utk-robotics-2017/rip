#include "path_follower_gui/arc.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            namespace gui
            {
                Arc::Arc(const Line& a, const Line& b)
                    : m_a(a)
                    , m_b(b)
                {
                    m_center = Line::intersect(m_a.end(), m_a.end().translateBy(m_a.slope().perpendicular()), m_b.start(), m_b.start().translateBy(m_b.slope().perpendicular()));
                    m_radius = (m_a.end() - m_center).norm();
                }

                void Arc::draw(QPainter& painter)
                {
                    m_a.draw(painter);
                    m_b.draw(painter);

                    Translation2d sTrans = m_a.end() - m_center;
                    Translation2d eTrans = m_b.start() - m_center;

                    units::Angle sAngle, eAngle;
                    if (Translation2d::cross(sTrans, eTrans) > 0)
                    {
                        eAngle = -atan2(sTrans.y()(), sTrans.x()()) * units::rad;
                        sAngle = -atan2(eTrans.y()(), eTrans.x()()) * units::rad;
                    }
                    else
                    {
                        sAngle = -atan2(sTrans.y()(), sTrans.x()()) * units::rad;
                        eAngle = -atan2(eTrans.y()(), eTrans.x()()) * units::rad;
                    }

                    double x = (m_center.x() - m_radius)();
                    double y = (m_center.y() - m_radius)();
                    double width = m_radius() * 2;
                    double height = m_radius() *  2;
                    int start_angle = sAngle.to(units::deg);
                    int end_angle = eAngle.to(units::deg);
                    int span_angle = end_angle - start_angle;
                    painter.drawArc(x, y, width, height, start_angle * 16, span_angle * 16);
                }

                Arc Arc::fromPoints(const Waypoint& a, const Waypoint& b, const Waypoint& c)
                {
                    return Arc(Line(a,b), Line(b,c));
                }
            }
        }
    }
}
