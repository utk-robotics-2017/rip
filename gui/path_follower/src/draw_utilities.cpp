#include "path_follower_gui/draw_utilities.hpp"
namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            namespace gui
            {
                void DrawUtilities::drawPolygon(QPainter& painter, const geometry::Polygon& polygon)
                {
                    QPainterPath path;
                    geometry::Point p = polygon.front();
                    path.moveTo(p.x()(), p.y()());
                    for (int i = 1; i <= polygon.size(); i++)
                    {
                        p = polygon[i % polygon.size()];
                        path.lineTo(p.x()(), p.y()());
                    }
                    painter.drawPath(path);
                }

                void DrawUtilities::drawArc(QPainter& painter, const Arc& arc)
                {
                    DrawUtilities::drawLine(painter, arc.a());
                    DrawUtilities::drawLine(painter, arc.b());

                    Translation2d sTrans = arc.center() - arc.a().end();
                    Translation2d eTrans = arc.center() - arc.b().start();

                    units::Angle sAngle, eAngle;
                    if(Translation2d::cross(sTrans, eTrans) > 0)
                    {
                        eAngle = -atan2(sTrans.y()(), sTrans.x()()) * units::rad;
                        sAngle = -atan2(eTrans.y()(), sTrans.x()()) * units::rad;
                    }
                    else
                    {
                        sAngle = -atan2(sTrans.y()(), sTrans.x()());
                        eAngle = -atan2(eTrans.y()(), sTrans.x()());
                    }

                    double x = (arc.center().x() - arc.radius())();
                    double y = (arc.center().y() - arc.radius())();
                    double width = arc.radius()() * 2;
                    double height = arc.radius()() *  2;
                    double start_angle = sAngle.to(units::deg) * 16;
                    double span_angle =  (eAngle - sAngle).to(units::deg) * 16;

                    painter.drawArc(x, y, width, height, start_angle, span_angle);
                }

                void DrawUtilities::drawLine(QPainter& painter, const Line& line)
                {
                    painter.drawLine(line.a().x()(), line.a().y()(), line.b().x()(), line.b().y()());
                }
            }
        }
    }
}
