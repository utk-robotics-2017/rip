#ifndef WAYPOINT_HPP
#define WAYPOINT_HPP

#include <units/units.hpp>

#include "path_follower/translation_2d.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            /**
             * A waypoint along a path. Contains a position, radius (for creating curved paths), and speed. The information from
             * these waypoints is used by the PathBuilder class to generate Paths. Waypoints also contain an optional marker
             * that is used by the WaitForPathMarkerAction.
             *
             * @see PathBuilder
             * @see WaitForPathMarkerAction
             */
            class Waypoint
            {
            public:
                Waypoint(const units::Distance& x, const units::Distance& y, const units::Distance& radius, const units::Velocity& speed);
                Waypoint(const units::Distance& x, const units::Distance& y, const units::Distance& radius, const units::Velocity& speed, const std::string& marker);
                Waypoint(const Translation2d& position, const units::Distance& radius, const units::Velocity& speed);
                Waypoint(const Waypoint& other);

                Translation2d position() const;
                units::Distance x() const;
                units::Distance y() const;
                units::Distance radius() const;
                units::Velocity speed() const;
                std::string marker() const;

            private:
                Translation2d m_position;
                units::Distance m_radius;
                units::Velocity m_speed;
                std::string m_marker;
            };
        }
    }
}

#endif //WAYPOINT_HPP
