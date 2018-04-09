#ifndef DRAW_UTILITIES_HPP
#define DRAW_UTILITIES_HPP

#include <QPainter>

#include <geometry/polygon.hpp>
#include <path_follower/arc.hpp>
namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            namespace gui
            {
                class DrawUtilities
                {
                public:
                    static void drawPolygon(QPainter& painter, const geometry::Polygon& polygon);
                    static void drawArc(QPainter& painter, const Arc& arc);
                    static void drawLine(QPainter& painter, const Line& line);
                };
            }
        }
    }
}

#endif // DRAW_UTILITIES_HPP
