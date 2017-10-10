#ifndef WAYPOINT_HPP
#define WAYPOINT_HPP

#include <units.hpp>
#include <json.hpp>

namespace rip
{
    namespace navigation
    {
        namespace pathfinder
        {
            struct Waypoint
            {
                Distance x;
                Distance y;
                Angle theta;
            };

            void from_json(const nlohmann::json& j, Waypoint& w)
            {
                w.x = j["x"];
                w.y = j["y"];
                w.theta = j["theta"];
            }

            void to_json(nlohmann::json& j, const Waypoint& w)
            {
                j = nlohmann::json
                {
                    {"x", w.x},
                    {"y", w.y},
                    {"theta", w.theta}
                };
            }
        }
    }
}

#endif
