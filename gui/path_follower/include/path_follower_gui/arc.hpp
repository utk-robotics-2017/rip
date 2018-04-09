#ifndef ARC_HPP
#define ARC_HPP

#include <QPainter>

#include "path_follower_gui/line.hpp"
#include "path_follower/waypoint.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            namespace gui
            {
                class Arc
                {
                public:
                    Arc(const Line& a, const Line& b);
                    void draw(QPainter& painter);
                    static Arc fromPoints(const Waypoint& a, const Waypoint& b, const Waypoint& c);
                private:
                   Line m_a;
                   Line m_b;
                   Translation2d m_center;
                   units::Distance m_radius;
                };
            }
        }
    }
}

#endif // ARC_HPP
