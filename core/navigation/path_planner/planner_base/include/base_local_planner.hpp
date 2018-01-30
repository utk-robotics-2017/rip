#ifndef BASE_LOCAL_PLANNER_HPP
#define BASE_LOCAL_PLANNER_HPP

namespace rip
{
    namespace navigation
    {
        namespace baseplanner
        {
            /**
             * @class BaseLocalPlanner
             * @brief Provides an interface for local planners used in navigation.
             * All local planners written as plugins for the navigation stack
             * must adhere to this interface.
             */
            class BaseLocalPlanner
            {
            public:
                /**
                 * Given the current position, orientation, and velocity of the robot,
                 * copmute the velocity commands to send
                 * @return [description]
                 */
                virtual VelocityPose computeVelocityCommands() = 0;

                virtual bool isGoalReached() = 0;
            };
        }
    }
}

#endif // BASE_LOCAL_PLANNER_HPP
