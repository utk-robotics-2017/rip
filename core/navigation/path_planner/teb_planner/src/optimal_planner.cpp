#include <teb_planner/optimal_planner.hpp>

#include <limits>

#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>

#include <teb_planner/edge_acceleration.hpp>
#include <teb_planner/edge_kinematics.hpp>
#include <teb_planner/edge_obstacle.hpp>
#include <teb_planner/edge_time_optimal.hpp>
#include <teb_planner/edge_velocity.hpp>
#include <teb_planner/edge_waypoint.hpp>

namespace rip
{
    namespace navigation
    {

        TebOptimalPlanner::TebOptimalPlanner(std::shared_ptr<TebConfig> config, const std::vector<std::shared_ptr<Obstacle> >& obstacles, std::shared_ptr<RobotFootprintModel> robot_model, const std::vector< geometry::Point >& waypoints)
            : m_config(config)
            , m_obstacles(obstacles)
            , m_robot_model(robot_model)
            , m_waypoints(waypoints)
        {
            m_optimizer = std::make_shared<g2o::SparseOptimizer>();

            using BlockSolver = g2o::BlockSolver < g2o::BlockSolverTraits < -1, -1 > >;
            using LinearSolver = g2o::LinearSolverCSparse< BlockSolver::PoseMatrixType >;

            std::unique_ptr<LinearSolver> linear_solver = std::unique_ptr<LinearSolver>(new LinearSolver);
            linear_solver->setBlockOrdering(true);
            std::unique_ptr<BlockSolver> block_solver = std::unique_ptr<BlockSolver>(new BlockSolver(std::move(linear_solver)));
            g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));
            m_optimizer->setAlgorithm(solver);
            m_optimizer->initMultiThreading();

        }

        void TebOptimalPlanner::plan(const Pose& start, const Pose& goal, const VelocityPose& start_velocity)
        {
            if (!m_teb.isInit())
            {
                m_teb.initTrajectoryToGoal(start, goal, 0, m_config->robot.max_velocity_x, m_config->trajectory.min_samples, m_config->trajectory.allow_init_with_backwards_motion);
            }
            else
            {
                if (m_teb.size() > 0 &&
                    goal.position().distance(m_teb.pose(m_teb.size() - 1).position()) < m_config->trajectory.force_reinit_new_goal_distance)
                {
                    m_teb.updateAndPrune(&start, &goal, m_config->trajectory.min_samples);
                }
                else
                {
                    m_teb.clear();
                    m_teb.initTrajectoryToGoal(start, goal, 0, m_config->robot.max_velocity_x, m_config->trajectory.min_samples, m_config->trajectory.allow_init_with_backwards_motion);
                }
            }

            m_start_velocity = start_velocity;

            optimizeTEB(m_config->optimization.num_inner_iterations, m_config->optimization.num_outer_iterations);
        }

        void TebOptimalPlanner::optimizeTEB(int iterations_innerloop, int iterations_outerloop)
        {
            double weight = 1.0;

            for (int i = 0; i < iterations_outerloop; i++)
            {
                if (m_config->trajectory.autosize)
                {
                    m_teb.autoResize(m_config->trajectory.dt_ref, m_config->trajectory.dt_hysteresis, m_config->trajectory.min_samples, m_config->trajectory.max_samples);
                }

                try
                {
                    buildGraph(weight);
                }
                catch (...)
                {
                    clear();
                    return;
                }

                try
                {
                    optimizeGraph(iterations_innerloop);
                }
                catch (...)
                {
                    clear();
                    return;
                }

                /*
                    if (compute_cost && i == iterations_outerloop - 1)
                    {
                        computeCurrentCost();
                    }
                    */

                clear();
                weight *= m_config->optimization.weight_adapt_factor;
            }
        }

        void TebOptimalPlanner::buildGraph(double weight)
        {
            clear();

            addVertices();

            // Add Edges (local cost functions)

            addEdgesObstacles(weight);

            // addEdgeDynamicObstacles(weight);

            addEdgesWaypoints();

            addEdgesVelocity();

            addEdgesAcceleration();

            addEdgesTimeOptimal();

            addEdgesKinematics();
        }

        void TebOptimalPlanner::optimizeGraph(int num_iterations)
        {
            if(m_config->robot.max_velocity_x < 0.01 * units::m / units::s)
            {
                clear();
                // todo: throw exception
            }

            if(!m_teb.isInit() || m_teb.size() < m_config->trajectory.min_samples)
            {
                clear();
                // todo: throw exception
            }

            m_optimizer->initializeOptimization();

            int iter = m_optimizer->optimize(num_iterations);

            if(!iter)
            {
                // todo: throw exception
            }

            clear();
        }

        void TebOptimalPlanner::clear()
        {
            m_optimizer->clear();
        }

        void TebOptimalPlanner::addVertices()
        {
            for (int i = 0, end = m_teb.size(); i < end; i++)
            {
                m_optimizer->addVertex(m_teb.poseVertex(i).get());
                if (m_teb.sizeTD() != 0 && i < m_teb.sizeTD())
                {
                    m_optimizer->addVertex(m_teb.timeDiffVertex(i).get());
                }
            }
        }

        void TebOptimalPlanner::addEdgesObstacles(double weight_multiplier)
        {
            if (m_config->optimization.obstacle_weight == 0 || weight_multiplier == 0 || m_obstacles.size() == 0)
            {
                return;
            }

            bool inflated = m_config->obstacles.inflation_distance > m_config->obstacles.min_obstacle_distance;

            Eigen::Matrix<double, 1, 1> information;
            information.fill(m_config->optimization.obstacle_weight * weight_multiplier);

            Eigen::Matrix<double, 2, 2> information_inflated;
            information_inflated(0, 0) = m_config->optimization.obstacle_weight * weight_multiplier;
            information_inflated(1, 1) = m_config->optimization.inflation_weight;
            information_inflated(0, 1) = information_inflated(1, 0) = 0;

            for (int i = 1, end = m_teb.size(); i < end; i++)
            {
                units::Distance left_min_distance = std::numeric_limits<double>::max();
                units::Distance right_min_distance = std::numeric_limits<double>::max();

                std::shared_ptr<Obstacle> left_obstacle = nullptr;
                std::shared_ptr<Obstacle> right_obstacle = nullptr;

                std::vector< std::shared_ptr<Obstacle> > relevant_obstacles;

                // Calculate distance to the current pose
                const geometry::Point pose_orientation = m_teb.pose(i).position();

                for (std::shared_ptr<Obstacle> obstacle : m_obstacles)
                {
                    if (obstacle->isDynamic())
                    {
                        continue;
                    }

                    // Calculate distance to current pose
                    units::Distance distance = obstacle->minimumDistance(m_teb.pose(i).position());

                    // Force considering obstacle if really close to the current pose
                    if (distance < m_config->obstacles.min_obstacle_distance * m_config->obstacles.obstacle_association_force_inclusion_factor)
                    {
                        relevant_obstacles.push_back(obstacle);
                        continue;
                    }

                    // Cut off distance
                    if (distance > m_config->obstacles.min_obstacle_distance * m_config->obstacles.obstacle_association_cutoff_factor)
                    {
                        continue;
                    }

                    // Determine side (left or right) and assign obstacle if closer than the previous one
                    if (pose_orientation.cross(obstacle->centroid()) > 0)
                    {
                        if (distance < left_min_distance)
                        {
                            left_min_distance = distance;
                            left_obstacle = obstacle;
                        }
                    }
                    else
                    {
                        if (distance < right_min_distance)
                        {
                            right_min_distance = distance;
                            right_obstacle = obstacle;
                        }
                    }
                }

                // Create obstacle edges
                if (left_obstacle)
                {
                    if (inflated)
                    {
                        EdgeInflatedObstacle* eio = new EdgeInflatedObstacle;
                        eio->setVertex(0, m_teb.poseVertex(i).get());
                        eio->setInformation(information_inflated);
                        eio->setParameters(m_config, m_robot_model, left_obstacle);
                        m_optimizer->addEdge(eio);
                    }
                    else
                    {
                        EdgeObstacle* eo = new EdgeObstacle;
                        eo->setVertex(0, m_teb.poseVertex(i).get());
                        eo->setInformation(information);
                        eo->setParameters(m_config, m_robot_model, left_obstacle);
                        m_optimizer->addEdge(eo);
                    }
                }

                if (right_obstacle)
                {
                    if (inflated)
                    {
                        EdgeInflatedObstacle* eio = new EdgeInflatedObstacle;
                        eio->setVertex(0, m_teb.poseVertex(i).get());
                        eio->setInformation(information_inflated);
                        eio->setParameters(m_config, m_robot_model, right_obstacle);
                        m_optimizer->addEdge(eio);
                    }
                    else
                    {
                        EdgeObstacle* eo = new EdgeObstacle;
                        eo->setVertex(0, m_teb.poseVertex(i).get());
                        eo->setInformation(information);
                        eo->setParameters(m_config, m_robot_model, right_obstacle);
                        m_optimizer->addEdge(eo);
                    }
                }

                for (std::shared_ptr<Obstacle> obstacle : relevant_obstacles)
                {
                    if (inflated)
                    {
                        EdgeInflatedObstacle* eio = new EdgeInflatedObstacle;
                        eio->setVertex(0, m_teb.poseVertex(i).get());
                        eio->setInformation(information_inflated);
                        eio->setParameters(m_config, m_robot_model, obstacle);
                        m_optimizer->addEdge(eio);
                    }
                    else
                    {
                        EdgeObstacle* eo = new EdgeObstacle;
                        eo->setVertex(0, m_teb.poseVertex(i).get());
                        eo->setInformation(information);
                        eo->setParameters(m_config, m_robot_model, obstacle);
                        m_optimizer->addEdge(eo);
                    }
                }
            }
        }

        void TebOptimalPlanner::addEdgeDynamicObstacles(double weight_multiplier)
        {
            if (m_config->optimization.obstacle_weight == 0 || weight_multiplier == 0 || m_obstacles.size() == 0)
            {
                return;
            }

            Eigen::Matrix<double, 2, 2> information;
            information(0, 0) = m_config->optimization.dynamic_obstacle_weight * weight_multiplier;
            information(1, 1) = m_config->optimization.dynamic_obstacle_inflation_weight * weight_multiplier;
            information(1, 0) = information(0, 1) = 0;

            for (std::shared_ptr<Obstacle> obstacle : m_obstacles)
            {
                if (!obstacle->isDynamic())
                {
                    continue;
                }

                // Skip the first and last pose as they are fixed
                units::Time time = m_teb.timeDiff(0);
                for (int i = 1, end = m_teb.size() - 1; i < end; i++)
                {
                    EdgeDynamicObstacle* edo = new EdgeDynamicObstacle(m_teb.sumAllTimeDiffsUpToIndex(i));
                    edo->setVertex(0, m_teb.poseVertex(i).get());
                    edo->setInformation(information);
                    edo->setParameters(m_config, m_robot_model, obstacle);
                    m_optimizer->addEdge(edo);
                    time += m_teb.timeDiff(i);
                }
            }
        }

        void TebOptimalPlanner::addEdgesWaypoints()
        {
            if (m_config->optimization.waypoint_weight == 0 || m_waypoints.size() == 0)
            {
                return;
            }

            int start_pose_idx = 0;

            int n = m_teb.size();

            // No degrees of freedom for reaching via-points
            if (n < 3)
            {
                return;
            }

            for (const geometry::Point& waypoint : m_waypoints)
            {
                int index = m_teb.findClosestTrajectoryPose(waypoint, nullptr, start_pose_idx);
                if (m_config->trajectory.waypoints_ordered)
                {
                    // Skip a point to have a DOF in between for further via-points
                    start_pose_idx = index + 1;
                }

                // Check if point coincides with goal or is located behind it
                if (index > n - 2)
                {
                    // Set to a pose before the goal, since we can move it away!
                    index = n - 2;
                }

                // Check if point coincides with start or is located before it
                if (index < 1)
                {
                    index = 1;
                }

                Eigen::Matrix<double, 1, 1> information;
                information.fill(m_config->optimization.waypoint_weight);

                EdgeWaypoint* ev = new EdgeWaypoint;
                ev->setVertex(0, m_teb.poseVertex(index).get());
                ev->setInformation(information);
                ev->setParameters(m_config, waypoint);
                m_optimizer->addEdge(ev);
            }
        }

        void TebOptimalPlanner::addEdgesVelocity()
        {
            // Non-holonomic robot
            if (m_config->robot.max_velocity_y == 0)
            {
                if (m_config->optimization.max_velocity_x_weight == 0 && m_config->optimization.max_velocity_theta_weight == 0)
                {
                    return;
                }

                int n = m_teb.size();
                Eigen::Matrix<double, 2, 2> information;
                information(0, 0) = m_config->optimization.max_velocity_x_weight;
                information(1, 1) = m_config->optimization.max_velocity_theta_weight;
                information(0, 1) = information(1, 0) = 0;

                for (int i = 0; i < n - 1; i++)
                {
                    EdgeVelocity* ve = new EdgeVelocity;
                    ve->setVertex(0, m_teb.poseVertex(i).get());
                    ve->setVertex(1, m_teb.poseVertex(i + 1).get());
                    ve->setVertex(2, m_teb.timeDiffVertex(i).get());
                    ve->setInformation(information);
                    ve->setConfig(m_config);
                    m_optimizer->addEdge(ve);
                }
            }
            // holonomic-robot
            else
            {
                if (m_config->optimization.max_velocity_x_weight == 0 && m_config->optimization.max_velocity_y_weight == 0 && m_config->optimization.max_velocity_theta_weight == 0)
                {
                    return;
                }

                int n = m_teb.size();
                Eigen::Matrix<double, 3, 3> information;
                information.fill(0);
                information(0, 0) = m_config->optimization.max_velocity_x_weight;
                information(1, 1) = m_config->optimization.max_velocity_y_weight;
                information(2, 2) = m_config->optimization.max_velocity_theta_weight;

                for (int i = 0; i < n - 1; i++)
                {
                    EdgeVelocityHolonomic* evh = new EdgeVelocityHolonomic;
                    evh->setVertex(0, m_teb.poseVertex(i).get());
                    evh->setVertex(1, m_teb.poseVertex(i + 1).get());
                    evh->setVertex(2, m_teb.timeDiffVertex(i).get());
                    evh->setInformation(information);
                    evh->setConfig(m_config);
                    m_optimizer->addEdge(evh);
                }
            }
        }

        void TebOptimalPlanner::addEdgesAcceleration()
        {
            if (m_config->optimization.acceleration_limit_x_weight == 0 && m_config->optimization.acceleration_limit_theta_weight == 0)
            {
                return;
            }

            int n = m_teb.size();

            // Non-holonomic
            if (m_config->robot.max_velocity_y == 0 || m_config->robot.acceleration_limit_y == 0)
            {
                Eigen::Matrix < double, 2, 2> information;
                information.fill(0);
                information(0, 0) = m_config->optimization.acceleration_limit_x_weight;
                information(1, 1) = m_config->optimization.acceleration_limit_theta_weight;

                // Chekc if an initial velocity should be taken into account
                if (m_start_velocity != VelocityPose(0, 0, 0))
                {
                    EdgeAccelerationStart* eas = new EdgeAccelerationStart;
                    eas->setVertex(0, m_teb.poseVertex(0).get());
                    eas->setVertex(1, m_teb.poseVertex(1).get());
                    eas->setVertex(2, m_teb.timeDiffVertex(0).get());
                    eas->setStartVelocity(m_start_velocity);
                    eas->setInformation(information);
                    eas->setConfig(m_config);
                    m_optimizer->addEdge(eas);
                }

                /**
                     * Add the usual acceleration edge for each tuple of three teb poses
                     */
                for (int i = 0; i < n - 2; i++)
                {
                    EdgeAcceleration* ea = new EdgeAcceleration;
                    ea->setVertex(0, m_teb.poseVertex(i).get());
                    ea->setVertex(1, m_teb.poseVertex(i + 1).get());
                    ea->setVertex(2, m_teb.poseVertex(i + 2).get());
                    ea->setVertex(3, m_teb.timeDiffVertex(i).get());
                    ea->setVertex(4, m_teb.timeDiffVertex(i + 1).get());
                    ea->setInformation(information);
                    ea->setConfig(m_config);
                    m_optimizer->addEdge(ea);
                }

                if (m_end_velocity != VelocityPose(0, 0, 0))
                {
                    EdgeAccelerationGoal* eag = new EdgeAccelerationGoal;
                    eag->setVertex(0, m_teb.poseVertex(n - 2).get());
                    eag->setVertex(1, m_teb.poseVertex(n - 1).get());
                    eag->setVertex(2, m_teb.timeDiffVertex(m_teb.sizeTD() - 1).get());
                    eag->setGoalVelocity(m_end_velocity);
                    eag->setInformation(information);
                    eag->setConfig(m_config);
                    m_optimizer->addEdge(eag);

                }
            }
            // Holonomic
            else
            {
                Eigen::Matrix<double, 3, 3> information;
                information.fill(0);
                information(0, 0) = m_config->optimization.acceleration_limit_x_weight;
                information(1, 1) = m_config->optimization.acceleration_limit_y_weight;
                information(2, 2) = m_config->optimization.acceleration_limit_theta_weight;

                // Chekc if an initial velocity should be taken into account
                if (m_start_velocity != VelocityPose(0, 0, 0))
                {
                    EdgeAccelerationHolonomicStart* eas = new EdgeAccelerationHolonomicStart;
                    eas->setVertex(0, m_teb.poseVertex(0).get());
                    eas->setVertex(1, m_teb.poseVertex(1).get());
                    eas->setVertex(2, m_teb.timeDiffVertex(0).get());
                    eas->setStartVelocity(m_start_velocity);
                    eas->setInformation(information);
                    eas->setConfig(m_config);
                    m_optimizer->addEdge(eas);
                }

                /**
                     * Add the usual acceleration edge for each tuple of three teb poses
                     */
                for (int i = 0; i < n - 2; i++)
                {
                    EdgeAccelerationHolonomic* ea = new EdgeAccelerationHolonomic;
                    ea->setVertex(0, m_teb.poseVertex(i).get());
                    ea->setVertex(1, m_teb.poseVertex(i + 1).get());
                    ea->setVertex(2, m_teb.poseVertex(i + 2).get());
                    ea->setVertex(3, m_teb.timeDiffVertex(i).get());
                    ea->setVertex(4, m_teb.timeDiffVertex(i + 1).get());
                    ea->setInformation(information);
                    ea->setConfig(m_config);
                    m_optimizer->addEdge(ea);
                }

                if (m_end_velocity != VelocityPose(0, 0, 0))
                {
                    EdgeAccelerationHolonomicGoal* eag = new EdgeAccelerationHolonomicGoal;
                    eag->setVertex(0, m_teb.poseVertex(n - 2).get());
                    eag->setVertex(1, m_teb.poseVertex(n - 1).get());
                    eag->setVertex(4, m_teb.timeDiffVertex(m_teb.sizeTD() - 1).get());
                    eag->setGoalVelocity(m_end_velocity);
                    eag->setInformation(information);
                    eag->setConfig(m_config);
                    m_optimizer->addEdge(eag);

                }
            }
        }

        void TebOptimalPlanner::addEdgesTimeOptimal()
        {
            if (m_config->optimization.optimal_time_weight == 0)
            {
                return;
            }

            Eigen::Matrix<double, 1, 1> information;
            information.fill(m_config->optimization.optimal_time_weight);

            for (int i = 0; i < m_teb.sizeTD(); i++)
            {
                EdgeTimeOptimal* eto = new EdgeTimeOptimal;
                eto->setVertex(0, m_teb.timeDiffVertex(i).get());
                eto->setInformation(information);
                eto->setConfig(m_config);
                m_optimizer->addEdge(eto);
            }
        }

        void TebOptimalPlanner::addEdgesKinematics()
        {
            if (m_config->optimization.kinematics_nh_weight == 0 && m_config->optimization.kinematics_forward_drive_weight == 0)
            {
                return;
            }

            // Create edge for satisfying kinematic constraints
            Eigen::Matrix<double, 2, 2> information;
            information.fill(0);
            information(0, 0) = m_config->optimization.kinematics_nh_weight;
            information(1, 1) = m_config->optimization.kinematics_forward_drive_weight;

            for (int i = 0; i < m_teb.size() - 1; i++)
            {
                EdgeKinematics* ek = new EdgeKinematics;
                ek->setVertex(0, m_teb.poseVertex(i).get());
                ek->setVertex(1, m_teb.poseVertex(i + 1).get());
                ek->setInformation(information);
                ek->setConfig(m_config);
                m_optimizer->addEdge(ek);
            }
        }

        std::vector<TrajectoryPoint> TebOptimalPlanner::getTrajectory() const
        {
            std::vector< TrajectoryPoint > trajectory;
            int n = m_teb.size();
            trajectory.resize(n);

            if (n == 0)
            {
                return trajectory;
            }

            units::Time time = 0;

            // Start
            TrajectoryPoint& start = trajectory.front();
            start.setPose(m_teb.pose(0));
            start.setVelocity(m_start_velocity);
            start.setTime(0);

            time += m_teb.timeDiff(0);

            for (int i = 1; i < n - 1; i++)
            {
                TrajectoryPoint& point = trajectory[i];
                point.setPose(m_teb.pose(i));
                VelocityPose velocity1 = extractVelocity(m_teb.pose(i - 1), m_teb.pose(i), m_teb.timeDiff(i - 1));
                VelocityPose velocity2 = extractVelocity(m_teb.pose(i), m_teb.pose(i + 1), m_teb.timeDiff(i));
                point.setVelocity((velocity1 + velocity2) / 2.0);
                point.setT(time);

                time += m_teb.timeDiff(i);
            }

            // Goal
            TrajectoryPoint& goal = trajectory.back();
            start.setPose(m_teb.pose(m_teb.size() - 1));
            start.setVelocity(m_end_velocity);
            start.setT(time);
        }

        bool TebOptimalPlanner::isTrajectoryFeasible() const
        {}

        bool TebOptimalPlanner::isHorzonReductionAppropriate() const
        {}

        VelocityPose TebOptimalPlanner::extractVelocity(const Pose& pose1, const Pose& pose2, const units::Time& dt) const
        {
            if(dt == 0)
            {
                return VelocityPose(0,0,0);
            }

            geometry::Point delta_s = pose2.position() - pose1.position();

            VelocityPose velocity;

            // Non-holonomic robot
            if(m_config->robot.max_velocity_y == 0)
            {
                geometry::Point conf1dir(units::cos(pose1.theta()), units::sin(pose1.theta()));

                double dir = delta_s.dot(conf1dir);
                velocity.setX(g2o::sign(dir) * delta_s.magnitude() / dt);
                velocity.setY(0);
            }
            else
            {
                double cos_theta1 = units::cos(pose1.theta());
                double sin_theta1 = units::sin(pose1.theta());
                velocity.setX((cos_theta1 * delta_s.x() + sin_theta1 * delta_s.y()) / dt);
                velocity.setY((-sin_theta1 * delta_s.x() + cos_theta1 * delta_s.y()) / dt);
            }

            velocity.setOmega(g2o::normalize_theta(pose2.theta().to(units::rad) - pose1.theta().to(units::rad)) * units::rad / dt);

            return velocity;
        }

    }
}
