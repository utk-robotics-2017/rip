#ifndef DRAW_UTILITIES_HPP
#define DRAW_UTILITIES_HPP

#include <QPainter>

#include <geometry/polygon.hpp>

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
                };
            }
        }
    }
}

#endif // DRAW_UTILITIES_HPP
