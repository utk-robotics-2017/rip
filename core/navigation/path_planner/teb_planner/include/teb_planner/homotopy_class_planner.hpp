#ifndef HOMOTOPY_CLASS_PLANNER_HPP
#define HOMOTOPY_CLASS_PLANNER_HPP

namespace rip
{
    namespace navigation
    {
        namespace tebplanner
        {
            /**
             * @class HomotopyClassPlanner
             * @brief Local planner that explores alternative homotopy classes, creates a plan for
             * each, and finally returns the robot controls for the best path
             *
             * Equivalence classes (e.g. homotopy) are explored using the help of a search-graph. \n
             * A couple of possible candidates are sampled / generated and filtered afterwards such that only a single candidate
             * per homotopy class remain. Filtering is applied using the H-Signature, a homotopy (resp. homology) invariant: \n
             *      - S. Bhattacharya et al.: Search-based Path Planning with Homotopy Class Constraints, AAAI, 2010
             *      - C. Rösmann et al.: Planning of Multiple Robot Trajectories in Distinctive Topologies, ECMR, 2015.
             *
             * Followed by the homotopy class search, each candidate is used as an initialization for the underlying trajectory
             * optimization (in this case utilizing the TebOptimalPlanner class with the TimedElasticBand). \n
             * Depending on the config parameters, the optimization is performed in parallel. \n
             * After the optimization is completed, the best optimized candidate is selected w.r.t. to trajectory cost, since the
             * cost already contains important features like clearance from obstacles and transition time. \n
             *
             * Everyhting is performed by calling one of the overloaded plan() methods. \n
             * Afterwards the velocity command to control the robot is obtained from the "best" candidate
             * via getVelocityCommand(). \n
             *
             * All steps are repeated in the subsequent sampling interval with the exception, that already planned (optimized) trajectories
             * are preferred against new path initilizations in order to improve the hot-starting capability.
             */
            class HomotopyClassPlanner : public PlannerInterface
            {
            public:
                using EquivalenceClassContainer = std::vector< std::pair< std::shared_ptr<EquivalenceClass>, bool > >;

                /**
                 * Defualt Constructor
                 */
                HomotopyClassPlanner();

                HomotopyClassPlanner(std::shared_ptr<TebConfig> config, const std::shared_ptr< std::shared_ptr<Obstacle> >& obstacles,
                                     std::shared_ptr< RobotModelFootprint > robot, const std::vector< geometry::Point >& waypoints);


            }
        }
    }
}

#endif // HOMOTOPY_CLASS_PLANNER_HPP