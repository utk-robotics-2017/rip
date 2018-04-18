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

            void to_json(nlohmann::json& j, const Lookahead& l)
            {
                j = {
                    {"min_distance", l.m_min_distance},
                    {"max_distance", l.m_max_distance},
                    {"min_speed", l.m_min_speed},
                    {"max_speed", l.m_max_speed}
                };
            }

            void from_json(const nlohmann::json& j, Lookahead& l)
            {
                l.m_min_distance = j["min_distance"];
                l.m_max_distance = j["max_distance"];
                l.m_min_speed = j["min_speed"];
                l.m_max_speed = j["max_speed"];
                l.m_delta_distance = l.m_max_distance - l.m_min_distance;
                l.m_delta_speed = l.m_max_speed - l.m_min_speed;
            }

        }
    }
}
