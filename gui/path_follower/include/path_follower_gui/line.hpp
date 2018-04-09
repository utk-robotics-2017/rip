#ifndef LINE_HPP
#define LINE_HPP

#include <QPainter>

#include "path_follower/translation_2d.hpp"
#include "path_follower/waypoint.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            namespace gui
            {
                class Line
                {
                public:
                    Line(const Waypoint& a, const Waypoint& b);
                    void draw(QPainter& painter);
                    static Translation2d intersect(const Translation2d& a, const Translation2d& b, const Translation2d& c, const Translation2d& d);

                    Translation2d slope() const;
                    Translation2d start() const;
                    Translation2d end() const;

                private:
                    Waypoint m_a;
                    Waypoint m_b;
                    Translation2d m_start;
                    Translation2d m_end;
                    Translation2d m_slope;
                    units::Velocity m_speed;
                };
            }
        }
    }
}

#endif // LINE_HPP
