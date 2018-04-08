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
            }
        }
    }
}