#ifndef BASE_GLOBAL_PLANNER_HPP
#define BASE_GLOBAL_PLANNER_HPP

namespace rip
{
    namespace navigation
    {
        namespace baseplanner
        {
            /**
             * @class BaseGlobalPlanner
             * Provides an interface for global planner used in navigation.
             * All global planners written as plugins for the navigation
             * stack must adhere to this interface.
             */
            class BaseGlobalPlanner
            {
                /**
                 * Given a goal pose in the world, compute a plan
                 *
                 * @param start The start pose
                 * @param goal The goal pose
                 * @returns The plan
                 */
                virtual std::vector< PoseStamped > makePlan(const PoseStamped& start, const PoseStamped& goal) = 0;

                /**
                 * Given a goal pose in the world, compute a plan
                 *
                 * @param start The start pose
                 * @param goal The goal pose
                 * @param cost The plan's calculated cost
                 * 2return The plan
                 */
                virtual std::vector< PoseStamped > makePlan(const PoseStamped& start, const PoseStamped& goal, double& cost)
                {
                    cost = 0;
                    return makePlan(start, goal);
                }

                /**
                 * Initialization
                 * @param name    The name of the planner
                 * @param costmap A pointer to the cost map used for planning
                 */
                virtual void intialize(const std::string& name, CostMap2D* costmap) = 0;

                /**
                 * Virtual destructor
                 */
                virtual ~BaseGlobalPlanner() {}

            protected:
                BaseGlobalPlanner() {}
            };
        }
    }
}

#endif // BASE_GLOBAL_PLANNER_HPP
