#ifndef LINE_HPP
#define LINE_HPP

#include "path_follower/path.hpp"
#include "path_follower/waypoint.hpp"
#include "path_follower/path_builder.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            class Line
            {
            public:
                Line(const Waypoint& a, const Waypoint& b);

                Waypoint a() const;
                Waypoint b() const;
                Translation2d start() const;
                Translation2d end() const;
                Translation2d slope() const;
                units::Velocity speed() const;

            private:
                void addToPath(Path& p, const units::Velocity& end_speed);
                friend class PathBuilder;
                friend class Arc;

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

#endif //LINE_HPP
