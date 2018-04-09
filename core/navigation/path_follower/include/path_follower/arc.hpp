#ifndef ARC_HPP
#define ARC_HPP

#include "path_follower/path.hpp"
#include "path_follower/waypoint.hpp"
#include "path_follower/translation_2d.hpp"
#include "path_follower/line.hpp"
#include "path_follower/path_builder.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            /**
             * An Arc object is formed by two Lines that share a common Waypoint. Contains a center position, radius, and speed.
             */
            class Arc
            {
            public:
                Arc(const Waypoint& a, const Waypoint& b, const Waypoint& c);
                Arc(const Line& a, const Line& b);

                Line a() const;
                Line b() const;
                Translation2d center() const;
                units::Distance radius() const;
                units::Velocity speed() const;

            private:
                static Translation2d intersect(const Line& l1, const Line& l2);
                void addToPath(Path& p);
                friend class PathBuilder;

                Line m_a;
                Line m_b;
                Translation2d m_center;
                units::Distance m_radius;
                units::Velocity m_speed;
            };
        }
    }
}

#endif //ARC_HPP
