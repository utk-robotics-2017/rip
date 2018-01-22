#ifndef OPTIMAL_PLANNER_HPP
#define OPTIMAL_PLANNER_HPP

#include <memory>
#include <vector>

#include <g2o/core/sparse_optimizer.h>

#include <geometry/point.hpp>

#include "planner_interface.hpp"
#include "teb_config.hpp"
#include "obstacle.hpp"
#include "robot_footprint_model.hpp"
#include "velocity_pose.hpp"
#include "pose.hpp"
#include "acceleration_pose.hpp"
#include "trajectory_point.hpp"
#include "timed_elastic_band.hpp"

namespace rip
{
    namespace navigation
    {
        class TebOptimalPlanner : public PlannerBase
        {
        public:
            /**
             * Construct and initialize the TEB optimal planner
             */
            TebOptimalPlanner(std::shared_ptr<TebConfig> config, const std::vector< std::shared_ptr<Obstacle> >& obstacles, std::shared_ptr<RobotFootprintModel> robot_model, const std::vector< geometry::Point >& waypoints);

            /**
             * Plan a trajectory between a given start and goal pose
             * @param start          [description]
             * @param goal           [description]
             * @param start_velocity [description]
             */
            virtual void plan(const Pose& start, const Pose& goal, const VelocityPose& start_velocity = VelocityPose()) override;

            std::vector< TrajectoryPoint > getTrajectory() const;
        private:
            /**
             * Optimize a previously initialized trajectory (acutal TEB optimization loop)
             *
             * Consists of two nested loops:
             *  - The outer loop resizes the trajectory according to the temporal resolution by invoking {@see TimeElasticBand#autoResize}
             *    Afterwards the internal method optimizeGraph() is called that constitutes the inner loop.
             *  - The inner loop calls the solver and iterates a specified number of optimization calls.
             * @param iterations_innerloop Number of iterations for the actual solver loop
             * @param iterations_outerloop Specifies how often trajectory should be resized followed by the inner solver loop
             */
            void optimizeTEB(int iterations_innerloop, int iterations_outerloop);

            /**
             * Builds the hyper graph representing the TEB optimization problem
             */
            void buildGraph(double weight);

            /**
             * Optimize the previously constructed hyper-graph to deform/optimize the TEB
             * @param num_iterations [description]
             */
            void optimizeGraph(int num_iterations);

            void clearGraph();

            /**
             * Add all relevant vertices to the hyper-graph as optimizable variables
             *
             * Vertices represent the variables that will be optimized.
             * In the case of the Timed Elastic Band poses and time differences form
             * vertices of the hyper-graph.
             * The order of insertion of the vertices (to the graph) is important
             * for efficiency, since it affects the sparsity pattern of the
             * underlying hessian computed for optimization.
             */
            void addVertices();

            /**
             * Add all edges (local cost function) related to keeping a distance
             * from static obstacles
             */
            void addEdgesObstacles(double weight_multiplier);

            /**
             * Add all edges (local cost function) related to keeping a distance
             * from dynamic obstacles
             */
            void addEdgeDynamicObstacles(double weight_multiplier);

            /**
             * Add all edges (local cost functions) related to minimizing the distance to
             * waypoints
             */
            void addEdgesWaypoints();

            /**
            * Add all edges (local cost functions) for limiting the
            * translational and angular velocity
            */
            void addEdgesVelocity();

            /**
             * Add all the edges (local cost functions) for limiting the translational and angular acceleration
             */
            void addEdgesAcceleration();

            /**
             * Add all edges (local cost functions) for minimizing the transition time
             */
            void addEdgesTimeOptimal();

            /**
             * Add add edges (local cost functions) for satisfying kinematic constraints
             */
            void addEdgesKinematics();

            /**
             * Check whether the planned trajectory is feasible or not.
             *
             * @note Currently only checks that the trajectory is collision free
             */
            bool isTrajectoryFeasible() const;

            /**
             * Check if the planner suggests a shorter horizon
             *
             * @note Only needed if the trajectory provided by the planner is infeasible
             */
            bool isHorzonReductionAppropriate() const;

            VelocityPose extractVelocity(const Pose& pose1, const Pose& pose2, const units::Time& dt) const;

            std::shared_ptr<TebConfig> m_config;
            std::vector<std::shared_ptr<Obstacle>> m_obstacles;
            std::shared_ptr<RobotFootprintModel> m_robot_model;
            std::vector< geometry::Point > m_waypoints;

            double m_cost;
            TimedElasticBand m_teb;
            std::shared_ptr<g2o::SparseOptimizer> m_optimizer;
            VelocityPose m_start_velocity;
            VelocityPose m_end_velocity;
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        };
    }
}

#endif // OPTIMAL_PLANNER_HPP
