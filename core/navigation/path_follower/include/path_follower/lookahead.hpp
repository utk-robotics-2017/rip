#ifndef LOOKAHEAD_HPP
#define LOOKAHEAD_HPP

#include <units/units.hpp>

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            /**
             * A utility class for interpolating lookahead distance based on current speed.
             */
            class Lookahead
            {
            public:
                Lookahead(const units::Distance& min_distance, const units::Distance& max_distance, const units::Velocity& min_speed, const units::Velocity& max_speed);

                units::Distance getLookaheadForSpeed(const units::Velocity& speed) const;
            private:
                units::Distance m_min_distance;
                units::Distance m_max_distance;
                units::Velocity m_min_speed;
                units::Velocity m_max_speed;
                units::Distance m_delta_distance;
                units::Velocity m_delta_speed;
            };
        }
    }
}

#endif //LOOKAHEAD_HPP
