#ifndef PATH_BUILDER_HPP
#define PATH_BUILDER_HPP

#include <vector>

#include "path_follower/path.hpp"
#include "path_follower/waypoint.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            class PathBuilder
            {
            public:
                static Path buildPathFromWaypoints(const std::vector<Waypoint>& waypoints);
                static Waypoint getPoint(const std::vector<Waypoint>& waypoints, int i);
            };
        }
    }
}

#endif //PATH_BUILDER_HPP
