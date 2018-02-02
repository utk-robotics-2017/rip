/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann
 *
 * Modified by: Andrew Messing
 * - Removed all aspects that required ROS or boost and added in RIP
 *   elements
 *********************************************************************/

#include <teb_planner/optimal_planner.hpp>
#include <map>
#include <limits>


namespace rip
{
    namespace navigation
    {
        namespace tebplanner
        {
            // ============== Implementation ===================

            TebOptimalPlanner::TebOptimalPlanner()
                : m_config(nullptr)
                , m_obstacles(nullptr)
                , via_points_(nullptr)
                , m_cost(HUGE_VAL)
                , prefer_rotdir_(RotType::none)
                , m_robot_model(new PointRobotFootprint())
                , m_initialized(false)
                , m_optimized(false)
            {
            }

            TebOptimalPlanner::TebOptimalPlanner(std::shared_ptr<TebConfig> cfg, ObstacleContainer* obstacles, RobotFootprintModelPtr robot_model, const ViaPointContainer* via_points)
            {
                initialize(cfg, obstacles, robot_model, via_points);
            }

            TebOptimalPlanner::~TebOptimalPlanner()
            {
                clearGraph();
            }

            void TebOptimalPlanner::initialize(std::shared_ptr<TebConfig> cfg, ObstacleContainer* obstacles, RobotFootprintModelPtr robot_model, const ViaPointContainer* via_points)
            {
                // init optimizer (set solver and block ordering settings)
                m_optimizer = initOptimizer();

                m_config = cfg;
                m_obstacles = obstacles;
                m_robot_model = robot_model;
                via_points_ = via_points;
                m_cost = HUGE_VAL;

                m_vel_start.first = true;
                m_vel_start.second.linear.x = 0;
                m_vel_start.second.linear.y = 0;
                m_vel_start.second.angular.z = 0;

                m_vel_goal.first = true;
                m_vel_goal.second.linear.x = 0;
                m_vel_goal.second.linear.y = 0;
                m_vel_goal.second.angular.z = 0;
                m_initialized = true;
            }

            /*
            * registers custom vertices and edges in g2o framework
            */
            void TebOptimalPlanner::registerG2OTypes()
            {
                g2o::Factory* factory = g2o::Factory::instance();
                factory->registerType("VERTEX_POSE", new g2o::HyperGraphElementCreator<VertexPose>);
                factory->registerType("VERTEX_TIMEDIFF", new g2o::HyperGraphElementCreator<VertexTimeDiff>);

                factory->registerType("EDGE_TIME_OPTIMAL", new g2o::HyperGraphElementCreator<EdgeTimeOptimal>);
                factory->registerType("EDGE_VELOCITY", new g2o::HyperGraphElementCreator<EdgeVelocity>);
                factory->registerType("EDGE_VELOCITY_HOLONOMIC", new g2o::HyperGraphElementCreator<EdgeVelocityHolonomic>);
                factory->registerType("EDGE_ACCELERATION", new g2o::HyperGraphElementCreator<EdgeAcceleration>);
                factory->registerType("EDGE_ACCELERATION_START", new g2o::HyperGraphElementCreator<EdgeAccelerationStart>);
                factory->registerType("EDGE_ACCELERATION_GOAL", new g2o::HyperGraphElementCreator<EdgeAccelerationGoal>);
                factory->registerType("EDGE_ACCELERATION_HOLONOMIC", new g2o::HyperGraphElementCreator<EdgeAccelerationHolonomic>);
                factory->registerType("EDGE_ACCELERATION_HOLONOMIC_START", new g2o::HyperGraphElementCreator<EdgeAccelerationHolonomicStart>);
                factory->registerType("EDGE_ACCELERATION_HOLONOMIC_GOAL", new g2o::HyperGraphElementCreator<EdgeAccelerationHolonomicGoal>);
                factory->registerType("EDGE_KINEMATICS_DIFF_DRIVE", new g2o::HyperGraphElementCreator<EdgeKinematicsDiffDrive>);
                factory->registerType("EDGE_KINEMATICS_CARLIKE", new g2o::HyperGraphElementCreator<EdgeKinematicsCarlike>);
                factory->registerType("EDGE_OBSTACLE", new g2o::HyperGraphElementCreator<EdgeObstacle>);
                factory->registerType("EDGE_INFLATED_OBSTACLE", new g2o::HyperGraphElementCreator<EdgeInflatedObstacle>);
                factory->registerType("EDGE_DYNAMIC_OBSTACLE", new g2o::HyperGraphElementCreator<EdgeDynamicObstacle>);
                //factory->registerType("EDGE_VIA_POINT", new g2o::HyperGraphElementCreator<EdgeViaPoint>);
                //factory->registerType("EDGE_PREFER_ROTDIR", new g2o::HyperGraphElementCreator<EdgePreferRotDir>);
                return;
            }

            /*
            * initialize g2o optimizer. Set solver settings here.
            * Return: pointer to new SparseOptimizer Object.
            */
            std::shared_ptr<g2o::SparseOptimizer> TebOptimalPlanner::initOptimizer()
            {
                // Call register_g2o_types once, even for multiple TebOptimalPlanner instances (thread-safe)
                static std::once_flag flag;
                std::call_once(flag, &registerG2OTypes);

                // allocating the optimizer
                std::shared_ptr<g2o::SparseOptimizer> optimizer = std::make_shared<g2o::SparseOptimizer>();

                std::unique_ptr<TEBLinearSolver> linear_solver = std::unique_ptr<TEBLinearSolver>(new TEBLinearSolver);
                linear_solver->setBlockOrdering(true);
                std::unique_ptr<TEBBlockSolver> block_solver = std::unique_ptr<TEBBlockSolver>(new TEBBlockSolver(std::move(linear_solver)));
                g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));
                optimizer->setAlgorithm(solver);

                optimizer->initMultiThreading(); // required for >Eigen 3.1

                return optimizer;
            }


            bool TebOptimalPlanner::optimizeTEB(int iterations_innerloop, int iterations_outerloop, bool compute_m_costafterwards,
                                                double obst_m_costscale, double viapoint_m_costscale, bool alternative_time_cost)
            {
                if (m_config->optim.optimization_activate == false)
                {
                    return false;
                }

                bool success = false;
                m_optimized = false;

                double weight_multiplier = 1.0;

                // TODO(roesmann): we introduced the non-fast mode with the support of dynamic obstacles
                //                (which leads to better results in terms of x-y-t homotopy planning).
                //                 however, we have not tested this mode intensively yet, so we keep
                //                 the legacy fast mode as default until we finish our tests.
                bool fast_mode = !m_config->obstacles.include_dynamic_obstacles;

                for (int i = 0; i < iterations_outerloop; ++i)
                {
                    if (m_config->trajectory.teb_autosize)
                    {
                        m_teb.autoResize(m_config->trajectory.dt_ref, m_config->trajectory.dt_hysteresis, m_config->trajectory.min_samples, m_config->trajectory.max_samples, fast_mode);

                    }

                    success = buildGraph(weight_multiplier);
                    if (!success)
                    {
                        clearGraph();
                        return false;
                    }
                    success = optimizeGraph(iterations_innerloop, false);
                    if (!success)
                    {
                        clearGraph();
                        return false;
                    }
                    m_optimized = true;

                    if (compute_m_costafterwards && i == iterations_outerloop - 1) // compute cost vec only in the last iteration
                    {
                        computeCurrentCost(obst_m_costscale, viapoint_m_costscale, alternative_time_cost);
                    }

                    clearGraph();

                    weight_multiplier *= m_config->optim.weight_adapt_factor;
                }

                return true;
            }

            void TebOptimalPlanner::setVelocityStart(const fakeros::Twist& vel_start)
            {
                m_vel_start.first = true;
                m_vel_start.second.linear.x = vel_start.linear.x;
                m_vel_start.second.linear.y = vel_start.linear.y;
                m_vel_start.second.angular.z = vel_start.angular.z;
            }

            void TebOptimalPlanner::setVelocityGoal(const fakeros::Twist& vel_goal)
            {
                m_vel_goal.first = true;
                m_vel_goal.second = vel_goal;
            }

            bool TebOptimalPlanner::plan(const std::vector<fakeros::PoseStamped>& initial_plan, const fakeros::Twist* start_vel, bool free_goal_vel)
            {
                if (!m_initialized)
                {
                    misc::Logger::getInstance()->error("Call initialize() first.");
                    assert(false);
                }

                if (!m_teb.isInit())
                {
                    // init trajectory
                    m_teb.initTrajectoryToGoal(initial_plan, m_config->robot.max_vel_x, m_config->trajectory.global_plan_overwrite_orientation, m_config->trajectory.min_samples, m_config->trajectory.allow_init_with_backwards_motion);
                }
                else // warm start
                {
                    PoseSE2 start_(initial_plan.front().pose);
                    PoseSE2 goal_(initial_plan.back().pose);
                    if (m_teb.sizePoses() > 0 && (goal_.position() - m_teb.backPose().position()).norm() < m_config->trajectory.force_reinit_new_goal_dist) // actual warm start!
                    {
                        m_teb.updateAndPruneTEB(&start_, &goal_, m_config->trajectory.min_samples);    // update TEB
                    }
                    else // goal too far away -> reinit
                    {
                        misc::Logger::getInstance()->debug("New goal: distance to existing goal is higher than the specified threshold. Reinitalizing trajectories.");
                        m_teb.clearTimedElasticBand();
                        m_teb.initTrajectoryToGoal(initial_plan, m_config->robot.max_vel_x, true, m_config->trajectory.min_samples, m_config->trajectory.allow_init_with_backwards_motion);
                    }
                }
                if (start_vel)
                {
                    setVelocityStart(*start_vel);
                }
                if (free_goal_vel)
                {
                    setVelocityGoalFree();
                }
                else
                {
                    m_vel_goal.first = true;    // we just reactivate and use the previously set velocity (should be zero if nothing was modified)
                }

                // now optimize
                return optimizeTEB(m_config->optim.no_inner_iterations, m_config->optim.no_outer_iterations);
            }

            bool TebOptimalPlanner::plan(const PoseSE2& start, const PoseSE2& goal, const fakeros::Twist* start_vel, bool free_goal_vel)
            {
                if (!m_initialized)
                {
                    misc::Logger::getInstance()->error("Call initialize() first.");
                    assert(false);
                }

                if (!m_teb.isInit())
                {
                    // init trajectory
                    m_teb.initTrajectoryToGoal(start, goal, 0, m_config->robot.max_vel_x, m_config->trajectory.min_samples, m_config->trajectory.allow_init_with_backwards_motion); // 0 intermediate samples, but dt=1 -> autoResize will add more samples before calling first optimization
                }
                else // warm start
                {
                    if (m_teb.sizePoses() > 0 && (goal.position() - m_teb.backPose().position()).norm() < m_config->trajectory.force_reinit_new_goal_dist) // actual warm start!
                    {
                        m_teb.updateAndPruneTEB(&start, &goal, m_config->trajectory.min_samples);
                    }
                    else // goal too far away -> reinit
                    {
                        misc::Logger::getInstance()->debug("New goal: distance to existing goal is higher than the specified threshold. Reinitalizing trajectories.");
                        m_teb.clearTimedElasticBand();
                        m_teb.initTrajectoryToGoal(start, goal, 0, m_config->robot.max_vel_x, m_config->trajectory.min_samples, m_config->trajectory.allow_init_with_backwards_motion);
                    }
                }
                if (start_vel)
                {
                    setVelocityStart(*start_vel);
                }
                if (free_goal_vel)
                {
                    setVelocityGoalFree();
                }
                else
                {
                    m_vel_goal.first = true;    // we just reactivate and use the previously set velocity (should be zero if nothing was modified)
                }

                // now optimize
                return optimizeTEB(m_config->optim.no_inner_iterations, m_config->optim.no_outer_iterations);
            }


            bool TebOptimalPlanner::buildGraph(double weight_multiplier)
            {
                if (!m_optimizer->edges().empty() || !m_optimizer->vertices().empty())
                {
                    misc::Logger::getInstance()->warn("Cannot build graph, because it is not empty. Call graphClear()!");
                    return false;
                }

                // add TEB vertices
                AddTEBVertices();

                // add Edges (local cost functions)
                if (m_config->obstacles.legacy_obstacle_association)
                {
                    AddEdgesObstaclesLegacy(weight_multiplier);
                }
                else
                {
                    AddEdgesObstacles(weight_multiplier);
                }

                if (m_config->obstacles.include_dynamic_obstacles)
                {
                    AddEdgesDynamicObstacles();
                }

                // AddEdgesViaPoints();

                AddEdgesVelocity();

                AddEdgesAcceleration();

                AddEdgesTimeOptimal();

                if (m_config->robot.min_turning_radius == 0 || m_config->optim.weight_kinematics_turning_radius == 0)
                {
                    AddEdgesKinematicsDiffDrive();    // we have a differential drive robot
                }
                else
                {
                    AddEdgesKinematicsCarlike();    // we have a carlike robot since the turning radius is bounded from below.
                }


                // AddEdgesPreferRotDir();

                return true;
            }

            bool TebOptimalPlanner::optimizeGraph(int no_iterations, bool clear_after)
            {
                if (m_config->robot.max_vel_x < 0.01)
                {
                    misc::Logger::getInstance()->warn("optimizeGraph(): Robot Max Velocity is smaller than 0.01m/s. Optimizing aborted...");
                    if (clear_after)
                    {
                        clearGraph();
                    }
                    return false;
                }

                if (!m_teb.isInit() || m_teb.sizePoses() < m_config->trajectory.min_samples)
                {
                    misc::Logger::getInstance()->warn("optimizeGraph(): TEB is empty or has too less elements. Skipping optimization.");
                    if (clear_after) { clearGraph(); }
                    return false;
                }

                m_optimizer->setVerbose(m_config->optim.optimization_verbose);
                m_optimizer->initializeOptimization();

                int iter = m_optimizer->optimize(no_iterations);

                // Save Hessian for visualization
                //  g2o::OptimizationAlgorithmLevenberg* lm = dynamic_cast<g2o::OptimizationAlgorithmLevenberg*> (m_optimizer->solver());
                //  lm->solver()->saveHessian("~/MasterThesis/Matlab/Hessian.txt");

                if (!iter)
                {
                    misc::Logger::getInstance()->error("optimizeGraph(): Optimization failed! iter=%i", iter);
                    return false;
                }

                if (clear_after) { clearGraph(); }

                return true;
            }

            void TebOptimalPlanner::clearGraph()
            {
                // clear optimizer states
                m_optimizer->vertices().clear();  // neccessary, because optimizer->clear deletes pointer-targets (therefore it deletes TEB states!)
                m_optimizer->clear();
            }



            void TebOptimalPlanner::AddTEBVertices()
            {
                // add vertices to graph
                misc::Logger::getInstance()->debug_if(m_config->optim.optimization_verbose, "Adding TEB vertices ...");
                unsigned int id_counter = 0; // used for vertices ids
                for (int i = 0; i < m_teb.sizePoses(); ++i)
                {
                    m_teb.poseVertex(i)->setId(id_counter++);
                    m_optimizer->addVertex(m_teb.poseVertex(i));
                    if (m_teb.sizeTimeDiffs() != 0 && i < m_teb.sizeTimeDiffs())
                    {
                        m_teb.timeDiffVertex(i)->setId(id_counter++);
                        m_optimizer->addVertex(m_teb.timeDiffVertex(i));
                    }
                }
            }


            void TebOptimalPlanner::AddEdgesObstacles(double weight_multiplier)
            {
                if (m_config->optim.weight_obstacle == 0 || weight_multiplier == 0 || m_obstacles == nullptr )
                {
                    return;    // if weight equals zero skip adding edges!
                }


                bool inflated = m_config->obstacles.inflation_dist > m_config->obstacles.min_obstacle_dist;

                Eigen::Matrix<double, 1, 1> information;
                information.fill(m_config->optim.weight_obstacle * weight_multiplier);

                Eigen::Matrix<double, 2, 2> information_inflated;
                information_inflated(0, 0) = m_config->optim.weight_obstacle * weight_multiplier;
                information_inflated(1, 1) = m_config->optim.weight_inflation;
                information_inflated(0, 1) = information_inflated(1, 0) = 0;

                // iterate all teb points (skip first and last)
                for (int i = 1; i < m_teb.sizePoses() - 1; ++i)
                {
                    double left_min_dist = std::numeric_limits<double>::max();
                    double right_min_dist = std::numeric_limits<double>::max();
                    std::shared_ptr<Obstacle> left_obstacle = nullptr;
                    std::shared_ptr<Obstacle> right_obstacle = nullptr;

                    std::vector< std::shared_ptr<Obstacle> > relevant_obstacles;

                    const Eigen::Vector2d pose_orient = m_teb.pose(i).orientationUnitVec();

                    // iterate obstacles
                    for (const ObstaclePtr& obst : *m_obstacles)
                    {
                        // we handle dynamic obstacles differently below
                        if (m_config->obstacles.include_dynamic_obstacles && obst->isDynamic())
                        {
                            continue;
                        }

                        // calculate distance to current pose
                        // TODO we ignore the robot footprint here in the association stage
                        double dist = obst->getMinimumDistance(m_teb.pose(i).position());

                        // force considering obstacle if really close to the current pose
                        if (dist < m_config->obstacles.min_obstacle_dist * m_config->obstacles.obstacle_association_force_inclusion_factor)
                        {
                            relevant_obstacles.push_back(obst);
                            continue;
                        }
                        // cut-off distance
                        if (dist > m_config->obstacles.min_obstacle_dist * m_config->obstacles.obstacle_association_cutoff_factor)
                        {
                            continue;
                        }

                        // determine side (left or right) and assign obstacle if closer than the previous one
                        if (cross2d(pose_orient, obst->getCentroid()) > 0) // left
                        {
                            if (dist < left_min_dist)
                            {
                                left_min_dist = dist;
                                left_obstacle = obst;
                            }
                        }
                        else
                        {
                            if (dist < right_min_dist)
                            {
                                right_min_dist = dist;
                                right_obstacle = obst;
                            }
                        }
                    }

                    // create obstacle edges
                    if (left_obstacle)
                    {
                        if (inflated)
                        {
                            EdgeInflatedObstacle* dist_bandpt_obst = new EdgeInflatedObstacle;
                            dist_bandpt_obst->setVertex(0, m_teb.poseVertex(i));
                            dist_bandpt_obst->setInformation(information_inflated);
                            dist_bandpt_obst->setParameters(m_config, m_robot_model, left_obstacle);
                            m_optimizer->addEdge(dist_bandpt_obst);
                        }
                        else
                        {
                            EdgeObstacle* dist_bandpt_obst = new EdgeObstacle;
                            dist_bandpt_obst->setVertex(0, m_teb.poseVertex(i));
                            dist_bandpt_obst->setInformation(information);
                            dist_bandpt_obst->setParameters(m_config, m_robot_model, left_obstacle);
                            m_optimizer->addEdge(dist_bandpt_obst);
                        }
                    }

                    if (right_obstacle)
                    {
                        if (inflated)
                        {
                            EdgeInflatedObstacle* dist_bandpt_obst = new EdgeInflatedObstacle;
                            dist_bandpt_obst->setVertex(0, m_teb.poseVertex(i));
                            dist_bandpt_obst->setInformation(information_inflated);
                            dist_bandpt_obst->setParameters(m_config, m_robot_model, right_obstacle);
                            m_optimizer->addEdge(dist_bandpt_obst);
                        }
                        else
                        {
                            EdgeObstacle* dist_bandpt_obst = new EdgeObstacle;
                            dist_bandpt_obst->setVertex(0, m_teb.poseVertex(i));
                            dist_bandpt_obst->setInformation(information);
                            dist_bandpt_obst->setParameters(m_config, m_robot_model, right_obstacle);
                            m_optimizer->addEdge(dist_bandpt_obst);
                        }
                    }

                    for (std::shared_ptr<Obstacle> obst : relevant_obstacles)
                    {
                        if (inflated)
                        {
                            EdgeInflatedObstacle* dist_bandpt_obst = new EdgeInflatedObstacle;
                            dist_bandpt_obst->setVertex(0, m_teb.poseVertex(i));
                            dist_bandpt_obst->setInformation(information_inflated);
                            dist_bandpt_obst->setParameters(m_config, m_robot_model, obst);
                            m_optimizer->addEdge(dist_bandpt_obst);
                        }
                        else
                        {
                            EdgeObstacle* dist_bandpt_obst = new EdgeObstacle;
                            dist_bandpt_obst->setVertex(0, m_teb.poseVertex(i));
                            dist_bandpt_obst->setInformation(information);
                            dist_bandpt_obst->setParameters(m_config, m_robot_model, obst);
                            m_optimizer->addEdge(dist_bandpt_obst);
                        }
                    }
                }

            }


            void TebOptimalPlanner::AddEdgesObstaclesLegacy(double weight_multiplier)
            {
                if (m_config->optim.weight_obstacle == 0 || weight_multiplier == 0 || m_obstacles == nullptr)
                {
                    return;    // if weight equals zero skip adding edges!
                }

                Eigen::Matrix<double, 1, 1> information;
                information.fill(m_config->optim.weight_obstacle * weight_multiplier);

                Eigen::Matrix<double, 2, 2> information_inflated;
                information_inflated(0, 0) = m_config->optim.weight_obstacle * weight_multiplier;
                information_inflated(1, 1) = m_config->optim.weight_inflation;
                information_inflated(0, 1) = information_inflated(1, 0) = 0;

                bool inflated = m_config->obstacles.inflation_dist > m_config->obstacles.min_obstacle_dist;

                for (ObstacleContainer::const_iterator obst = m_obstacles->begin(); obst != m_obstacles->end(); ++obst)
                {
                    if (m_config->obstacles.include_dynamic_obstacles && (*obst)->isDynamic()) // we handle dynamic obstacles differently below
                    {
                        continue;
                    }

                    int index;

                    if (m_config->obstacles.obstacle_poses_affected >= m_teb.sizePoses())
                    {
                        index =  m_teb.sizePoses() / 2;
                    }
                    else
                    {
                        index = m_teb.findClosestTrajectoryPose(*(obst->get()));
                    }


                    // check if obstacle is outside index-range between start and goal
                    if ( (index <= 1) || (index > m_teb.sizePoses() - 2) ) // start and goal are fixed and findNearestBandpoint finds first or last conf if intersection point is outside the range
                    {
                        continue;
                    }

                    if (inflated)
                    {
                        EdgeInflatedObstacle* dist_bandpt_obst = new EdgeInflatedObstacle;
                        dist_bandpt_obst->setVertex(0, m_teb.poseVertex(index));
                        dist_bandpt_obst->setInformation(information_inflated);
                        dist_bandpt_obst->setParameters(m_config, m_robot_model, *obst);
                        m_optimizer->addEdge(dist_bandpt_obst);
                    }
                    else
                    {
                        EdgeObstacle* dist_bandpt_obst = new EdgeObstacle;
                        dist_bandpt_obst->setVertex(0, m_teb.poseVertex(index));
                        dist_bandpt_obst->setInformation(information);
                        dist_bandpt_obst->setParameters(m_config, m_robot_model, *obst);
                        m_optimizer->addEdge(dist_bandpt_obst);
                    }

                    for (int neighbourIdx = 0; neighbourIdx < floor(m_config->obstacles.obstacle_poses_affected / 2); neighbourIdx++)
                    {
                        if (index + neighbourIdx < m_teb.sizePoses())
                        {
                            if (inflated)
                            {
                                EdgeInflatedObstacle* dist_bandpt_obst_n_r = new EdgeInflatedObstacle;
                                dist_bandpt_obst_n_r->setVertex(0, m_teb.poseVertex(index + neighbourIdx));
                                dist_bandpt_obst_n_r->setInformation(information_inflated);
                                dist_bandpt_obst_n_r->setParameters(m_config, m_robot_model, *obst);
                                m_optimizer->addEdge(dist_bandpt_obst_n_r);
                            }
                            else
                            {
                                EdgeObstacle* dist_bandpt_obst_n_r = new EdgeObstacle;
                                dist_bandpt_obst_n_r->setVertex(0, m_teb.poseVertex(index + neighbourIdx));
                                dist_bandpt_obst_n_r->setInformation(information);
                                dist_bandpt_obst_n_r->setParameters(m_config, m_robot_model, *obst);
                                m_optimizer->addEdge(dist_bandpt_obst_n_r);
                            }
                        }
                        if ( index - neighbourIdx >= 0) // needs to be casted to int to allow negative values
                        {
                            if (inflated)
                            {
                                EdgeInflatedObstacle* dist_bandpt_obst_n_l = new EdgeInflatedObstacle;
                                dist_bandpt_obst_n_l->setVertex(0, m_teb.poseVertex(index - neighbourIdx));
                                dist_bandpt_obst_n_l->setInformation(information_inflated);
                                dist_bandpt_obst_n_l->setParameters(m_config, m_robot_model, *obst);
                                m_optimizer->addEdge(dist_bandpt_obst_n_l);
                            }
                            else
                            {
                                EdgeObstacle* dist_bandpt_obst_n_l = new EdgeObstacle;
                                dist_bandpt_obst_n_l->setVertex(0, m_teb.poseVertex(index - neighbourIdx));
                                dist_bandpt_obst_n_l->setInformation(information);
                                dist_bandpt_obst_n_l->setParameters(m_config, m_robot_model, *obst);
                                m_optimizer->addEdge(dist_bandpt_obst_n_l);
                            }
                        }
                    }

                }
            }


            void TebOptimalPlanner::AddEdgesDynamicObstacles(double weight_multiplier)
            {
                if (m_config->optim.weight_obstacle == 0 || weight_multiplier == 0 || m_obstacles == nullptr )
                {
                    return;    // if weight equals zero skip adding edges!
                }

                Eigen::Matrix<double, 2, 2> information;
                information(0, 0) = m_config->optim.weight_dynamic_obstacle * weight_multiplier;
                information(1, 1) = m_config->optim.weight_dynamic_obstacle_inflation;
                information(0, 1) = information(1, 0) = 0;

                for (ObstacleContainer::const_iterator obst = m_obstacles->begin(); obst != m_obstacles->end(); ++obst)
                {
                    if (!(*obst)->isDynamic())
                    {
                        continue;
                    }

                    // Skip first and last pose, as they are fixed
                    double time = m_teb.timeDiff(0);
                    for (int i = 1; i < m_teb.sizePoses() - 1; ++i)
                    {
                        EdgeDynamicObstacle* dynobst_edge = new EdgeDynamicObstacle(m_teb.getSumOfTimeDiffsUpToIdx(i));
                        dynobst_edge->setVertex(0, m_teb.poseVertex(i));
                        dynobst_edge->setInformation(information);
                        dynobst_edge->setParameters(m_config, m_robot_model, *obst);
                        m_optimizer->addEdge(dynobst_edge);
                        time += m_teb.timeDiff(i); // we do not need to check the time diff bounds, since we iterate to "< sizePoses()-1".
                    }
                }
            }
#if 0
            void TebOptimalPlanner::AddEdgesViaPoints()
            {
                if (m_config->optim.weight_viapoint == 0 || via_points_ == nullptr || via_points_->empty() )
                {
                    return;    // if weight equals zero skip adding edges!
                }

                int start_pose_idx = 0;

                int n = m_teb.sizePoses();
                if (n < 3) // we do not have any degrees of freedom for reaching via-points
                {
                    return;
                }

                for (ViaPointContainer::const_iterator vp_it = via_points_->begin(); vp_it != via_points_->end(); ++vp_it)
                {

                    int index = m_teb.findClosestTrajectoryPose(*vp_it, NULL, start_pose_idx);
                    if (m_config->trajectory.via_points_ordered)
                    {
                        start_pose_idx = index + 2;    // skip a point to have a DOF inbetween for further via-points
                    }

                    // check if point conicides with goal or is located behind it
                    if ( index > n - 2 )
                    {
                        index = n - 2;    // set to a pose before the goal, since we can move it away!
                    }
                    // check if point coincides with start or is located before it
                    if ( index < 1)
                    {
                        index = 1;
                    }

                    Eigen::Matrix<double, 1, 1> information;
                    information.fill(m_config->optim.weight_viapoint);

                    EdgeViaPoint* edge_viapoint = new EdgeViaPoint;
                    edge_viapoint->setVertex(0, m_teb.PoseVertex(index));
                    edge_viapoint->setInformation(information);
                    edge_viapoint->setParameters(m_config, &(*vp_it));
                    m_optimizer->addEdge(edge_viapoint);
                }
            }
#endif
            void TebOptimalPlanner::AddEdgesVelocity()
            {
                if (m_config->robot.max_vel_y == 0) // non-holonomic robot
                {
                    if ( m_config->optim.weight_max_vel_x == 0 && m_config->optim.weight_max_vel_theta == 0)
                    {
                        return;    // if weight equals zero skip adding edges!
                    }

                    int n = m_teb.sizePoses();
                    Eigen::Matrix<double, 2, 2> information;
                    information(0, 0) = m_config->optim.weight_max_vel_x;
                    information(1, 1) = m_config->optim.weight_max_vel_theta;
                    information(0, 1) = 0.0;
                    information(1, 0) = 0.0;

                    for (int i = 0; i < n - 1; ++i)
                    {
                        EdgeVelocity* velocity_edge = new EdgeVelocity;
                        velocity_edge->setVertex(0, m_teb.poseVertex(i));
                        velocity_edge->setVertex(1, m_teb.poseVertex(i + 1));
                        velocity_edge->setVertex(2, m_teb.timeDiffVertex(i));
                        velocity_edge->setInformation(information);
                        velocity_edge->setTebConfig(m_config);
                        m_optimizer->addEdge(velocity_edge);
                    }
                }
                else // holonomic-robot
                {
                    if ( m_config->optim.weight_max_vel_x == 0 && m_config->optim.weight_max_vel_y == 0 && m_config->optim.weight_max_vel_theta == 0)
                    {
                        return;    // if weight equals zero skip adding edges!
                    }

                    int n = m_teb.sizePoses();
                    Eigen::Matrix<double, 3, 3> information;
                    information.fill(0);
                    information(0, 0) = m_config->optim.weight_max_vel_x;
                    information(1, 1) = m_config->optim.weight_max_vel_y;
                    information(2, 2) = m_config->optim.weight_max_vel_theta;

                    for (int i = 0; i < n - 1; ++i)
                    {
                        EdgeVelocityHolonomic* velocity_edge = new EdgeVelocityHolonomic;
                        velocity_edge->setVertex(0, m_teb.poseVertex(i));
                        velocity_edge->setVertex(1, m_teb.poseVertex(i + 1));
                        velocity_edge->setVertex(2, m_teb.timeDiffVertex(i));
                        velocity_edge->setInformation(information);
                        velocity_edge->setTebConfig(m_config);
                        m_optimizer->addEdge(velocity_edge);
                    }

                }
            }

            void TebOptimalPlanner::AddEdgesAcceleration()
            {
                if (m_config->optim.weight_acc_lim_x == 0  && m_config->optim.weight_acc_lim_theta == 0)
                {
                    return;    // if weight equals zero skip adding edges!
                }

                int n = m_teb.sizePoses();

                if (m_config->robot.max_vel_y == 0 || m_config->robot.acc_lim_y == 0) // non-holonomic robot
                {
                    Eigen::Matrix<double, 2, 2> information;
                    information.fill(0);
                    information(0, 0) = m_config->optim.weight_acc_lim_x;
                    information(1, 1) = m_config->optim.weight_acc_lim_theta;

                    // check if an initial velocity should be taken into accound
                    if (m_vel_start.first)
                    {
                        EdgeAccelerationStart* acceleration_edge = new EdgeAccelerationStart;
                        acceleration_edge->setVertex(0, m_teb.poseVertex(0));
                        acceleration_edge->setVertex(1, m_teb.poseVertex(1));
                        acceleration_edge->setVertex(2, m_teb.timeDiffVertex(0));
                        acceleration_edge->setInitialVelocity(m_vel_start.second);
                        acceleration_edge->setInformation(information);
                        acceleration_edge->setTebConfig(m_config);
                        m_optimizer->addEdge(acceleration_edge);
                    }

                    // now add the usual acceleration edge for each tuple of three teb poses
                    for (int i = 0; i < n - 2; ++i)
                    {
                        EdgeAcceleration* acceleration_edge = new EdgeAcceleration;
                        acceleration_edge->setVertex(0, m_teb.poseVertex(i));
                        acceleration_edge->setVertex(1, m_teb.poseVertex(i + 1));
                        acceleration_edge->setVertex(2, m_teb.poseVertex(i + 2));
                        acceleration_edge->setVertex(3, m_teb.timeDiffVertex(i));
                        acceleration_edge->setVertex(4, m_teb.timeDiffVertex(i + 1));
                        acceleration_edge->setInformation(information);
                        acceleration_edge->setTebConfig(m_config);
                        m_optimizer->addEdge(acceleration_edge);
                    }

                    // check if a goal velocity should be taken into accound
                    if (m_vel_goal.first)
                    {
                        EdgeAccelerationGoal* acceleration_edge = new EdgeAccelerationGoal;
                        acceleration_edge->setVertex(0, m_teb.poseVertex(n - 2));
                        acceleration_edge->setVertex(1, m_teb.poseVertex(n - 1));
                        acceleration_edge->setVertex(2, m_teb.timeDiffVertex( m_teb.sizeTimeDiffs() - 1 ));
                        acceleration_edge->setGoalVelocity(m_vel_goal.second);
                        acceleration_edge->setInformation(information);
                        acceleration_edge->setTebConfig(m_config);
                        m_optimizer->addEdge(acceleration_edge);
                    }
                }
                else // holonomic robot
                {
                    Eigen::Matrix<double, 3, 3> information;
                    information.fill(0);
                    information(0, 0) = m_config->optim.weight_acc_lim_x;
                    information(1, 1) = m_config->optim.weight_acc_lim_y;
                    information(2, 2) = m_config->optim.weight_acc_lim_theta;

                    // check if an initial velocity should be taken into accound
                    if (m_vel_start.first)
                    {
                        EdgeAccelerationHolonomicStart* acceleration_edge = new EdgeAccelerationHolonomicStart;
                        acceleration_edge->setVertex(0, m_teb.poseVertex(0));
                        acceleration_edge->setVertex(1, m_teb.poseVertex(1));
                        acceleration_edge->setVertex(2, m_teb.timeDiffVertex(0));
                        acceleration_edge->setInitialVelocity(m_vel_start.second);
                        acceleration_edge->setInformation(information);
                        acceleration_edge->setTebConfig(m_config);
                        m_optimizer->addEdge(acceleration_edge);
                    }

                    // now add the usual acceleration edge for each tuple of three teb poses
                    for (int i = 0; i < n - 2; ++i)
                    {
                        EdgeAccelerationHolonomic* acceleration_edge = new EdgeAccelerationHolonomic;
                        acceleration_edge->setVertex(0, m_teb.poseVertex(i));
                        acceleration_edge->setVertex(1, m_teb.poseVertex(i + 1));
                        acceleration_edge->setVertex(2, m_teb.poseVertex(i + 2));
                        acceleration_edge->setVertex(3, m_teb.timeDiffVertex(i));
                        acceleration_edge->setVertex(4, m_teb.timeDiffVertex(i + 1));
                        acceleration_edge->setInformation(information);
                        acceleration_edge->setTebConfig(m_config);
                        m_optimizer->addEdge(acceleration_edge);
                    }

                    // check if a goal velocity should be taken into accound
                    if (m_vel_goal.first)
                    {
                        EdgeAccelerationHolonomicGoal* acceleration_edge = new EdgeAccelerationHolonomicGoal;
                        acceleration_edge->setVertex(0, m_teb.poseVertex(n - 2));
                        acceleration_edge->setVertex(1, m_teb.poseVertex(n - 1));
                        acceleration_edge->setVertex(2, m_teb.timeDiffVertex( m_teb.sizeTimeDiffs() - 1 ));
                        acceleration_edge->setGoalVelocity(m_vel_goal.second);
                        acceleration_edge->setInformation(information);
                        acceleration_edge->setTebConfig(m_config);
                        m_optimizer->addEdge(acceleration_edge);
                    }
                }
            }



            void TebOptimalPlanner::AddEdgesTimeOptimal()
            {
                if (m_config->optim.weight_optimaltime == 0)
                {
                    return;    // if weight equals zero skip adding edges!
                }

                Eigen::Matrix<double, 1, 1> information;
                information.fill(m_config->optim.weight_optimaltime);

                for (int i = 0; i < m_teb.sizeTimeDiffs(); ++i)
                {
                    EdgeTimeOptimal* timeoptimal_edge = new EdgeTimeOptimal;
                    timeoptimal_edge->setVertex(0, m_teb.timeDiffVertex(i));
                    timeoptimal_edge->setInformation(information);
                    timeoptimal_edge->setTebConfig(m_config);
                    m_optimizer->addEdge(timeoptimal_edge);
                }
            }



            void TebOptimalPlanner::AddEdgesKinematicsDiffDrive()
            {
                if (m_config->optim.weight_kinematics_nh == 0 && m_config->optim.weight_kinematics_forward_drive == 0)
                {
                    return;    // if weight equals zero skip adding edges!
                }

                // create edge for satisfiying kinematic constraints
                Eigen::Matrix<double, 2, 2> information_kinematics;
                information_kinematics.fill(0.0);
                information_kinematics(0, 0) = m_config->optim.weight_kinematics_nh;
                information_kinematics(1, 1) = m_config->optim.weight_kinematics_forward_drive;

                for (int i = 0; i < m_teb.sizePoses() - 1; i++) // ignore twiced start only
                {
                    EdgeKinematicsDiffDrive* kinematics_edge = new EdgeKinematicsDiffDrive;
                    kinematics_edge->setVertex(0, m_teb.poseVertex(i));
                    kinematics_edge->setVertex(1, m_teb.poseVertex(i + 1));
                    kinematics_edge->setInformation(information_kinematics);
                    kinematics_edge->setTebConfig(m_config);
                    m_optimizer->addEdge(kinematics_edge);
                }
            }

            void TebOptimalPlanner::AddEdgesKinematicsCarlike()
            {
                if (m_config->optim.weight_kinematics_nh == 0 && m_config->optim.weight_kinematics_turning_radius)
                {
                    return;    // if weight equals zero skip adding edges!
                }

                // create edge for satisfiying kinematic constraints
                Eigen::Matrix<double, 2, 2> information_kinematics;
                information_kinematics.fill(0.0);
                information_kinematics(0, 0) = m_config->optim.weight_kinematics_nh;
                information_kinematics(1, 1) = m_config->optim.weight_kinematics_turning_radius;

                for (int i = 0; i < m_teb.sizePoses() - 1; i++) // ignore twiced start only
                {
                    EdgeKinematicsCarlike* kinematics_edge = new EdgeKinematicsCarlike;
                    kinematics_edge->setVertex(0, m_teb.poseVertex(i));
                    kinematics_edge->setVertex(1, m_teb.poseVertex(i + 1));
                    kinematics_edge->setInformation(information_kinematics);
                    kinematics_edge->setTebConfig(m_config);
                    m_optimizer->addEdge(kinematics_edge);
                }
            }

#if 0
            void TebOptimalPlanner::AddEdgesPreferRotDir()
            {
                if (prefer_rotdir_ == RotType::none || m_config->optim.weight_prefer_rotdir == 0)
                {
                    return;    // if weight equals zero skip adding edges!
                }

                // create edge for satisfiying kinematic constraints
                Eigen::Matrix<double, 1, 1> information_rotdir;
                information_rotdir.fill(m_config->optim.weight_prefer_rotdir);

                for (int i = 0; i < m_teb.sizePoses() - 1 && i < 3; ++i) // currently: apply to first 3 rotations
                {
                    EdgePreferRotDir* rotdir_edge = new EdgePreferRotDir;
                    rotdir_edge->setVertex(0, m_teb.poseVertex(i));
                    rotdir_edge->setVertex(1, m_teb.poseVertex(i + 1));
                    rotdir_edge->setInformation(information_rotdir);

                    if (prefer_rotdir_ == RotType::left)
                    {
                        rotdir_edge->preferLeft();
                    }
                    else
                    {
                        rotdir_edge->preferRight();
                    }

                    m_optimizer->addEdge(rotdir_edge);
                }
            }
#endif
            void TebOptimalPlanner::computeCurrentCost(double obst_m_costscale, double viapoint_m_costscale, bool alternative_time_cost)
            {
                // check if graph is empty/exist  -> important if function is called between buildGraph and optimizeGraph/clearGraph
                bool graph_exist_flag(false);
                if (m_optimizer->edges().empty() && m_optimizer->vertices().empty())
                {
                    // here the graph is build again, for time efficiency make sure to call this function
                    // between buildGraph and Optimize (deleted), but it depends on the application
                    buildGraph();
                    m_optimizer->initializeOptimization();
                }
                else
                {
                    graph_exist_flag = true;
                }

                m_optimizer->computeInitialGuess();

                m_cost = 0;

                if (alternative_time_cost)
                {
                    m_cost += m_teb.getSumOfAllTimeDiffs();
                    // TEST we use SumOfAllTimeDiffs() here, because edge cost depends on number of samples, which is not always the same for similar TEBs,
                    // since we are using an AutoResize Function with hysteresis.
                }

                // now we need pointers to all edges -> calculate error for each edge-type
                // since we aren't storing edge pointers, we need to check every edge
                for (std::vector<g2o::OptimizableGraph::Edge*>::const_iterator it = m_optimizer->activeEdges().begin(); it != m_optimizer->activeEdges().end(); it++)
                {
                    EdgeTimeOptimal* edge_time_optimal = dynamic_cast<EdgeTimeOptimal*>(*it);
                    if (edge_time_optimal != nullptr && !alternative_time_cost)
                    {
                        m_cost += edge_time_optimal->getError().squaredNorm();
                        continue;
                    }

                    EdgeKinematicsDiffDrive* edge_kinematics_dd = dynamic_cast<EdgeKinematicsDiffDrive*>(*it);
                    if (edge_kinematics_dd != nullptr)
                    {
                        m_cost += edge_kinematics_dd->getError().squaredNorm();
                        continue;
                    }

                    EdgeKinematicsCarlike* edge_kinematics_cl = dynamic_cast<EdgeKinematicsCarlike*>(*it);
                    if (edge_kinematics_cl != nullptr)
                    {
                        m_cost += edge_kinematics_cl->getError().squaredNorm();
                        continue;
                    }

                    EdgeVelocity* edge_velocity = dynamic_cast<EdgeVelocity*>(*it);
                    if (edge_velocity != nullptr)
                    {
                        m_cost += edge_velocity->getError().squaredNorm();
                        continue;
                    }

                    EdgeAcceleration* edge_acceleration = dynamic_cast<EdgeAcceleration*>(*it);
                    if (edge_acceleration != nullptr)
                    {
                        m_cost += edge_acceleration->getError().squaredNorm();
                        continue;
                    }

                    EdgeObstacle* edge_obstacle = dynamic_cast<EdgeObstacle*>(*it);
                    if (edge_obstacle != nullptr)
                    {
                        m_cost += edge_obstacle->getError().squaredNorm() * obst_m_costscale;
                        continue;
                    }

                    EdgeInflatedObstacle* edge_inflated_obstacle = dynamic_cast<EdgeInflatedObstacle*>(*it);
                    if (edge_inflated_obstacle != nullptr)
                    {
                        m_cost += std::sqrt(std::pow(edge_inflated_obstacle->getError()[0], 2) * obst_m_costscale
                                            + std::pow(edge_inflated_obstacle->getError()[1], 2));
                        continue;
                    }

                    EdgeDynamicObstacle* edge_dyn_obstacle = dynamic_cast<EdgeDynamicObstacle*>(*it);
                    if (edge_dyn_obstacle != nullptr)
                    {
                        m_cost += edge_dyn_obstacle->getError().squaredNorm() * obst_m_costscale;
                        continue;
                    }
#if 0
                    EdgeViaPoint* edge_viapoint = dynamic_cast<EdgeViaPoint*>(*it);
                    if (edge_viapoint != nullptr)
                    {
                        m_cost += edge_viapoint->getError().squaredNorm() * viapoint_m_costscale;
                        continue;
                    }
#endif
                }

                // delete temporary created graph
                if (!graph_exist_flag)
                {
                    clearGraph();
                }
            }


            void TebOptimalPlanner::extractVelocity(const PoseSE2& pose1, const PoseSE2& pose2, double dt, double& vx, double& vy, double& omega) const
            {
                if (dt == 0)
                {
                    vx = 0;
                    vy = 0;
                    omega = 0;
                    return;
                }

                Eigen::Vector2d deltaS = pose2.position() - pose1.position();

                if (m_config->robot.max_vel_y == 0) // nonholonomic robot
                {
                    Eigen::Vector2d conf1dir( cos(pose1.theta()), sin(pose1.theta()) );
                    // translational velocity
                    double dir = deltaS.dot(conf1dir);
                    vx = (double) g2o::sign(dir) * deltaS.norm() / dt;
                    vy = 0;
                }
                else // holonomic robot
                {
                    // transform pose 2 into the current robot frame (pose1)
                    // for velocities only the rotation of the direction vector is necessary.
                    // (map->pose1-frame: inverse 2d rotation matrix)
                    double cos_theta1 = std::cos(pose1.theta());
                    double sin_theta1 = std::sin(pose1.theta());
                    double p1_dx =  cos_theta1 * deltaS.x() + sin_theta1 * deltaS.y();
                    double p1_dy = -sin_theta1 * deltaS.x() + cos_theta1 * deltaS.y();
                    vx = p1_dx / dt;
                    vy = p1_dy / dt;
                }

                // rotational velocity
                double orientdiff = g2o::normalize_theta(pose2.theta() - pose1.theta());
                omega = orientdiff / dt;
            }

            bool TebOptimalPlanner::getVelocityCommand(double& vx, double& vy, double& omega) const
            {
                if (m_teb.sizePoses() < 2)
                {
                    misc::Logger::getInstance()->error("TebOptimalPlanner::getVelocityCommand(): The trajectory contains less than 2 poses. Make sure to init and optimize/plan the trajectory fist.");
                    vx = 0;
                    vy = 0;
                    omega = 0;
                    return false;
                }

                double dt = m_teb.timeDiff(0);
                if (dt <= 0)
                {
                    misc::Logger::getInstance()->error("TebOptimalPlanner::getVelocityCommand() - timediff<=0 is invalid!");
                    vx = 0;
                    vy = 0;
                    omega = 0;
                    return false;
                }

                // Get velocity from the first two configurations
                extractVelocity(m_teb.pose(0), m_teb.pose(1), dt, vx, vy, omega);
                return true;
            }

            void TebOptimalPlanner::getVelocityProfile(std::vector<fakeros::Twist>& velocity_profile) const
            {
                int n = m_teb.sizePoses();
                velocity_profile.resize( n + 1 );

                // start velocity
                velocity_profile.front().linear.z = 0;
                velocity_profile.front().angular.x = velocity_profile.front().angular.y = 0;
                velocity_profile.front().linear.x = m_vel_start.second.linear.x;
                velocity_profile.front().linear.y = m_vel_start.second.linear.y;
                velocity_profile.front().angular.z = m_vel_start.second.angular.z;

                for (int i = 1; i < n; ++i)
                {
                    velocity_profile[i].linear.z = 0;
                    velocity_profile[i].angular.x = velocity_profile[i].angular.y = 0;
                    extractVelocity(m_teb.pose(i - 1), m_teb.pose(i), m_teb.timeDiff(i - 1), velocity_profile[i].linear.x, velocity_profile[i].linear.y, velocity_profile[i].angular.z);
                }

                // goal velocity
                velocity_profile.back().linear.z = 0;
                velocity_profile.back().angular.x = velocity_profile.back().angular.y = 0;
                velocity_profile.back().linear.x = m_vel_goal.second.linear.x;
                velocity_profile.back().linear.y = m_vel_goal.second.linear.y;
                velocity_profile.back().angular.z = m_vel_goal.second.angular.z;
            }

            void TebOptimalPlanner::getFullTrajectory(std::vector<fakeros::TrajectoryPointMsg>& trajectory) const
            {
                int n = m_teb.sizePoses();

                trajectory.resize(n);

                if (n == 0)
                {
                    return;
                }

                double curr_time = 0;

                // start
                fakeros::TrajectoryPointMsg& start = trajectory.front();
                m_teb.pose(0).toPoseMsg(start.pose);
                start.velocity.linear.z = 0;
                start.velocity.angular.x = start.velocity.angular.y = 0;
                start.velocity.linear.x = m_vel_start.second.linear.x;
                start.velocity.linear.y = m_vel_start.second.linear.y;
                start.velocity.angular.z = m_vel_start.second.angular.z;
                start.time_from_start.sec = static_cast<int>(curr_time);
                start.time_from_start.nsec = static_cast<int>((curr_time - start.time_from_start.sec) * 10E6);

                curr_time += m_teb.timeDiff(0);

                // intermediate points
                for (int i = 1; i < n - 1; ++i)
                {
                    fakeros::TrajectoryPointMsg& point = trajectory[i];
                    m_teb.pose(i).toPoseMsg(point.pose);
                    point.velocity.linear.z = 0;
                    point.velocity.angular.x = point.velocity.angular.y = 0;
                    double vel1_x, vel1_y, vel2_x, vel2_y, omega1, omega2;
                    extractVelocity(m_teb.pose(i - 1), m_teb.pose(i), m_teb.timeDiff(i - 1), vel1_x, vel1_y, omega1);
                    extractVelocity(m_teb.pose(i), m_teb.pose(i + 1), m_teb.timeDiff(i), vel2_x, vel2_y, omega2);
                    point.velocity.linear.x = 0.5 * (vel1_x + vel2_x);
                    point.velocity.linear.y = 0.5 * (vel1_y + vel2_y);
                    point.velocity.angular.z = 0.5 * (omega1 + omega2);
                    point.time_from_start.sec = static_cast<int>(curr_time);
                    point.time_from_start.nsec = static_cast<int>((curr_time - point.time_from_start.sec) * 10E6);

                    curr_time += m_teb.timeDiff(i);
                }

                // goal
                fakeros::TrajectoryPointMsg& goal = trajectory.back();
                m_teb.backPose().toPoseMsg(goal.pose);
                goal.velocity.linear.z = 0;
                goal.velocity.angular.x = goal.velocity.angular.y = 0;
                goal.velocity.linear.x = m_vel_goal.second.linear.x;
                goal.velocity.linear.y = m_vel_goal.second.linear.y;
                goal.velocity.angular.z = m_vel_goal.second.angular.z;
                goal.time_from_start.sec = static_cast<int>(curr_time);
                goal.time_from_start.nsec = static_cast<int>((curr_time - goal.time_from_start.sec) * 10E6);
            }

#if 0
            bool TebOptimalPlanner::isTrajectoryFeasible(base_local_planner::CostmapModel* costmap_model, const std::vector<fakeros::Point>& footprint_spec,
                    double inscribed_radius, double circumscribed_radius, int look_ahead_idx)
            {
                if (look_ahead_idx < 0 || look_ahead_idx >= teb().sizePoses())
                {
                    look_ahead_idx = teb().sizePoses() - 1;
                }

                for (int i = 0; i <= look_ahead_idx; ++i)
                {
                    if ( costmap_model->footprintCost(teb().Pose(i).x(), teb().Pose(i).y(), teb().Pose(i).theta(), footprint_spec, inscribed_radius, circumscribed_radius) < 0 )
                    {
                        return false;
                    }

                    // check if distance between two poses is higher than the robot radius and interpolate in that case
                    // (if obstacles are pushing two consecutive poses away, the center between two consecutive poses might coincide with the obstacle ;-)!
                    if (i < look_ahead_idx)
                    {
                        if ( (teb().Pose(i + 1).position() - teb().Pose(i).position()).norm() > inscribed_radius)
                        {
                            // check one more time
                            PoseSE2 center = PoseSE2::average(teb().Pose(i), teb().Pose(i + 1));
                            if ( costmap_model->footprintCost(center.x(), center.y(), center.theta(), footprint_spec, inscribed_radius, circumscribed_radius) < 0 )
                            {
                                return false;
                            }
                        }

                    }
                }
                return true;
            }
#endif

            bool TebOptimalPlanner::isHorizonReductionAppropriate(const std::vector<fakeros::PoseStamped>& initial_plan) const
            {
                if (m_teb.sizePoses() < int( 1.5 * double(m_config->trajectory.min_samples) ) ) // trajectory is short already
                {
                    return false;
                }

                // check if distance is at least 2m long // hardcoded for now
                double dist = 0;
                for (int i = 1; i < m_teb.sizePoses(); ++i)
                {
                    dist += ( m_teb.pose(i).position() - m_teb.pose(i - 1).position() ).norm();
                    if (dist > 2)
                    {
                        break;
                    }
                }
                if (dist <= 2)
                {
                    return false;
                }

                // check if goal orientation is differing with more than 90° and the horizon is still long enough to exclude parking maneuvers.
                // use case: Sometimes the robot accomplish the following navigation task:
                // 1. wall following 2. 180° curve 3. following along the other side of the wall.
                // If the trajectory is too long, the trajectory might intersect with the obstace and the optimizer does
                // push the trajectory to the correct side.
                if ( std::abs( g2o::normalize_theta( m_teb.pose(0).theta() - m_teb.backPose().theta() ) ) > M_PI / 2)
                {
                    misc::Logger::getInstance()->debug("TebOptimalPlanner::isHorizonReductionAppropriate(): Goal orientation - start orientation > 90° ");
                    return true;
                }

                // check if goal heading deviates more than 90° w.r.t. start orienation
                if (m_teb.pose(0).orientationUnitVec().dot(m_teb.backPose().position() - m_teb.pose(0).position()) < 0)
                {
                    misc::Logger::getInstance()->debug("TebOptimalPlanner::isHorizonReductionAppropriate(): Goal heading - start orientation > 90° ");
                    return true;
                }

                // check ratio: distance along the inital plan and distance of the trajectory (maybe too much is cut off)
                int idx = 0; // first get point close to the robot (should be fast if the global path is already pruned!)
                for (; idx < (int)initial_plan.size(); ++idx)
                {
                    if ( std::sqrt(std::pow(initial_plan[idx].pose.position.x - m_teb.pose(0).x(), 2) + std::pow(initial_plan[idx].pose.position.y - m_teb.pose(0).y(), 2)) )
                    {
                        break;
                    }
                }
                // now calculate length
                double ref_path_length = 0;
                for (; idx < int(initial_plan.size()) - 1; ++idx)
                {
                    ref_path_length += std::sqrt(std::pow(initial_plan[idx + 1].pose.position.x - initial_plan[idx].pose.position.x, 2)
                                                 + std::pow(initial_plan[idx + 1].pose.position.y - initial_plan[idx].pose.position.y, 2) );
                }

                // check distances along the teb trajectory (by the way, we also check if the distance between two poses is > obst_dist)
                double m_teblength = 0;
                for (int i = 1; i < m_teb.sizePoses(); ++i )
                {
                    double dist = (m_teb.pose(i).position() - m_teb.pose(i - 1).position()).norm();
                    if (dist > 0.95 * m_config->obstacles.min_obstacle_dist)
                    {
                        misc::Logger::getInstance()->debug("TebOptimalPlanner::isHorizonReductionAppropriate(): Distance between consecutive poses > 0.9*min_obstacle_dist");
                        return true;
                    }
                    ref_path_length += dist;
                }
                if (ref_path_length > 0 && m_teblength / ref_path_length < 0.7) // now check ratio
                {
                    misc::Logger::getInstance()->debug("TebOptimalPlanner::isHorizonReductionAppropriate(): Planned trajectory is at least 30° shorter than the initial plan");
                    return true;
                }


                // otherwise we do not suggest shrinking the horizon:
                return false;
            }

        }
    }
}
