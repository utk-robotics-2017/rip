#ifndef PATH_CONTAINER_HPP
#define PATH_CONTAINER_HPP

#include "path_follower/path.hpp"
#include "path_follower/rigid_transform_2d.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            /**
             * Interface containing all information necessary for a path including the Path itself, the Path's starting pose, and
             * whether or not the robot should drive in reverse along the path.
             */
            class PathContainer
            {
            public:
                virtual Path buildPath() = 0;

                virtual RigidTransform2d getStartPose() = 0;

                virtual bool isReversed() = 0;
            };
        }
    }
}

#endif //PATH_CONTAINER_HPP
