#include <misc/logger.hpp>
#include "path_follower/path_builder.hpp"
#include "path_follower/arc.hpp"
#include "path_follower/exceptions.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            Path PathBuilder::buildPathFromWaypoints(const std::vector<Waypoint>& waypoints)
            {
                Path p;
                if(waypoints.size() < 2)
                {
                    misc::Logger::getInstance()->error("Path must contain at least 2 waypoints");
                    throw PathBuildingException("Path must contain at least 2 waypoints");
                }

                int i = 0;
                if(waypoints.size() > 2)
                {
                    do
                    {
                        Arc(getPoint(waypoints, i), getPoint(waypoints, i + 1), getPoint(waypoints, i + 2)).addToPath(p);
                        i++;
                    }while(i < waypoints.size() - 2);
                }
                Line(waypoints[waypoints.size() - 2], waypoints[waypoints.size() - 1]).addToPath(p, 0);
                p.extrapolateLast();
                p.verifySpeeds();
                return p;
            }
        }
    }
}