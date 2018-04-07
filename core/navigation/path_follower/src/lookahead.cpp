#include "path_follower/lookahead.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            Lookahead::Lookahead(const units::Distance& min_distance, const units::Distance& max_distance, const units::Velocity& min_speed, const units::Velocity& max_speed)
                : m_min_distance(min_distance)
                , m_max_distance(max_distance)
                , m_min_speed(min_speed)
                , m_max_speed(max_speed)
            {
                m_delta_distance = m_max_distance - m_min_distance;
                m_delta_speed = m_max_speed - m_min_speed;
            }

            units::Distance Lookahead::getLookaheadForSpeed(const units::Velocity& speed) const
            {
                units::Distance lookahead = m_delta_distance * (speed - m_min_speed) / m_delta_speed + m_min_distance;
                return units::isnan(lookahead) ? m_min_distance : units::max(m_min_distance, units::min(m_max_distance, lookahead));
            }
        }
    }
}