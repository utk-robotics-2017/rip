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

#include <teb_planner/teb_config.hpp>

#include <misc/logger.hpp>

namespace rip
{
    namespace navigation
    {
        namespace tebplanner
        {
            void TebConfig::checkParameters() const
            {
                // positive backward velocity?
                if (robot.max_vel_x_backwards <= 0)
                {
                    misc::Logger::getInstance()->warn("TebLocalPlannerROS() Param Warning: Do not choose max_vel_x_backwards to be <=0. Disable backwards driving by increasing the optimization weight for penalyzing backwards driving.");
                }

                // bounds smaller than penalty epsilon
                if (robot.max_vel_x <= optim.penalty_epsilon)
                {
                    misc::Logger::getInstance()->warn("TebLocalPlannerROS() Param Warning: max_vel_x <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");
                }

                if (robot.max_vel_x_backwards <= optim.penalty_epsilon)
                {
                    misc::Logger::getInstance()->warn("TebLocalPlannerROS() Param Warning: max_vel_x_backwards <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");
                }

                if (robot.max_vel_theta <= optim.penalty_epsilon)
                {
                    misc::Logger::getInstance()->warn("TebLocalPlannerROS() Param Warning: max_vel_theta <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");
                }

                if (robot.acc_lim_x <= optim.penalty_epsilon)
                {
                    misc::Logger::getInstance()->warn("TebLocalPlannerROS() Param Warning: acc_lim_x <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");
                }

                if (robot.acc_lim_theta <= optim.penalty_epsilon)
                {
                    misc::Logger::getInstance()->warn("TebLocalPlannerROS() Param Warning: acc_lim_theta <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");
                }

                // dt_ref and dt_hyst
                if (trajectory.dt_ref <= trajectory.dt_hysteresis)
                {
                    misc::Logger::getInstance()->warn("TebLocalPlannerROS() Param Warning: dt_ref <= dt_hysteresis. The hysteresis is not allowed to be greater or equal!. Undefined behavior... Change at least one of them!");
                }

                // min number of samples
                if (trajectory.min_samples < 3)
                {
                    misc::Logger::getInstance()->warn("TebLocalPlannerROS() Param Warning: parameter min_samples is smaller than 3! Sorry, I haven't enough degrees of freedom to plan a trajectory for you. Please increase ...");
                }

                // costmap obstacle behind robot
                if (obstacles.costmap_obstacles_behind_robot_dist < 0)
                {
                    misc::Logger::getInstance()->warn("TebLocalPlannerROS() Param Warning: parameter 'costmap_obstacles_behind_robot_dist' should be positive or zero.");
                }

                // hcp: obstacle heading threshold
                if (hcp.obstacle_keypoint_offset >= 1 || hcp.obstacle_keypoint_offset <= 0)
                {
                    misc::Logger::getInstance()->warn("TebLocalPlannerROS() Param Warning: parameter obstacle_heading_threshold must be in the interval ]0,1[. 0=0deg opening angle, 1=90deg opening angle.");
                }

                // carlike
                if (robot.cmd_angle_instead_rotvel && robot.wheelbase == 0)
                {
                    misc::Logger::getInstance()->warn("TebLocalPlannerROS() Param Warning: parameter cmd_angle_instead_rotvel is non-zero but wheelbase is set to zero: undesired behavior.");
                }

                if (robot.cmd_angle_instead_rotvel && robot.min_turning_radius == 0)
                {
                    misc::Logger::getInstance()->warn("TebLocalPlannerROS() Param Warning: parameter cmd_angle_instead_rotvel is non-zero but min_turning_radius is set to zero: undesired behavior. You are mixing a carlike and a diffdrive robot");
                }

                // positive weight_adapt_factor
                if (optim.weight_adapt_factor < 1.0)
                {
                    misc::Logger::getInstance()->warn("TebLocalPlannerROS() Param Warning: parameter weight_adapt_factor shoud be >= 1.0");
                }

                if (recovery.oscillation_filter_duration < 0)
                {
                    misc::Logger::getInstance()->warn("TebLocalPlannerROS() Param Warning: parameter oscillation_filter_duration must be >= 0");
                }
            }

#define TO_JSON(sub, var) j[#var] = c.sub.var;

            void to_json(nlohmann::json& j, const TebConfig& c)
            {
                // Trajectory
                TO_JSON(trajectory, teb_autosize);
                TO_JSON(trajectory, dt_ref);
                TO_JSON(trajectory, dt_hysteresis);
                TO_JSON(trajectory, min_samples);
                TO_JSON(trajectory, max_samples);
                TO_JSON(trajectory, global_plan_overwrite_orientation);
                TO_JSON(trajectory, allow_init_with_backwards_motion);
                TO_JSON(trajectory, global_plan_viapoint_sep);
                TO_JSON(trajectory, via_points_ordered);
                TO_JSON(trajectory, max_global_plan_lookahead_dist);
                TO_JSON(trajectory, exact_arc_length);
                TO_JSON(trajectory, force_reinit_new_goal_dist);
                TO_JSON(trajectory, feasibility_check_no_poses);
                TO_JSON(trajectory, publish_feedback);

                // Robot
                TO_JSON(robot, max_vel_x);
                TO_JSON(robot, max_vel_x_backwards);
                TO_JSON(robot, max_vel_y);
                TO_JSON(robot, max_vel_theta);
                TO_JSON(robot, acc_lim_x);
                TO_JSON(robot, acc_lim_y);
                TO_JSON(robot, acc_lim_theta);
                TO_JSON(robot, min_turning_radius);
                TO_JSON(robot, wheelbase);
                TO_JSON(robot, cmd_angle_instead_rotvel);
                TO_JSON(robot, is_footprint_dynamic);

                // Goal Tolerance
                TO_JSON(goal_tolerance, xy_goal_tolerance);
                TO_JSON(goal_tolerance, yaw_goal_tolerance);
                TO_JSON(goal_tolerance, free_goal_vel);

                // Obstacles

                TO_JSON(obstacles, min_obstacle_dist);
                TO_JSON(obstacles, inflation_dist);
                TO_JSON(obstacles, dynamic_obstacle_inflation_dist);
                TO_JSON(obstacles, include_dynamic_obstacles);
                TO_JSON(obstacles, include_costmap_obstacles);
                TO_JSON(obstacles, costmap_obstacles_behind_robot_dist);
                TO_JSON(obstacles, obstacle_poses_affected);
                TO_JSON(obstacles, legacy_obstacle_association);
                TO_JSON(obstacles, obstacle_association_force_inclusion_factor);
                TO_JSON(obstacles, obstacle_association_cutoff_factor);
                TO_JSON(obstacles, costmap_converter_plugin);
                TO_JSON(obstacles, costmap_converter_spin_thread);
                TO_JSON(obstacles, costmap_converter_rate);

                // Optimization
                TO_JSON(optim, no_inner_iterations);
                TO_JSON(optim, no_outer_iterations);
                TO_JSON(optim, optimization_activate);
                TO_JSON(optim, optimization_verbose);
                TO_JSON(optim, penalty_epsilon );
                TO_JSON(optim, weight_max_vel_x);
                TO_JSON(optim, weight_max_vel_y);
                TO_JSON(optim, weight_max_vel_theta);
                TO_JSON(optim, weight_acc_lim_x);
                TO_JSON(optim, weight_acc_lim_y);
                TO_JSON(optim, weight_acc_lim_theta);
                TO_JSON(optim, weight_kinematics_nh);
                TO_JSON(optim, weight_kinematics_forward_drive);
                TO_JSON(optim, weight_kinematics_turning_radius);
                TO_JSON(optim, weight_optimaltime);
                TO_JSON(optim, weight_obstacle);
                TO_JSON(optim, weight_inflation);
                TO_JSON(optim, weight_dynamic_obstacle);
                TO_JSON(optim, weight_dynamic_obstacle_inflation);
                TO_JSON(optim, weight_viapoint);
                TO_JSON(optim, weight_prefer_rotdir);
                TO_JSON(optim, weight_adapt_factor);

                // Homotopy Class Planner
                TO_JSON(hcp, enable_homotopy_class_planning);
                TO_JSON(hcp, enable_multithreading);
                TO_JSON(hcp, simple_exploration);
                TO_JSON(hcp, max_number_classes);
                TO_JSON(hcp, selection_cost_hysteresis);
                TO_JSON(hcp, selection_prefer_initial_plan);
                TO_JSON(hcp, selection_obst_cost_scale);
                TO_JSON(hcp, selection_viapoint_cost_scale);
                TO_JSON(hcp, selection_alternative_time_cost);
                TO_JSON(hcp, obstacle_keypoint_offset);
                TO_JSON(hcp, obstacle_heading_threshold);
                TO_JSON(hcp, roadmap_graph_no_samples);
                TO_JSON(hcp, roadmap_graph_area_width); // [m]
                TO_JSON(hcp, roadmap_graph_area_length_scale);
                TO_JSON(hcp, h_signature_prescaler);
                TO_JSON(hcp, h_signature_threshold);
                TO_JSON(hcp, viapoints_all_candidates);
                TO_JSON(hcp, visualize_hc_graph);
                TO_JSON(hcp, visualize_with_time_as_z_axis_scale);

                // Recovery
                TO_JSON(recovery, shrink_horizon_backup);
                TO_JSON(recovery, shrink_horizon_min_duration);
                TO_JSON(recovery, oscillation_recovery);
                TO_JSON(recovery, oscillation_v_eps);
                TO_JSON(recovery, oscillation_omega_eps);
                TO_JSON(recovery, oscillation_recovery_min_duration);
                TO_JSON(recovery, oscillation_filter_duration);
            }

#define FROM_JSON(sub, var) c.sub.var = j[#var];

            void from_json(const nlohmann::json& j, TebConfig& c)
            {
                // Trajectory
                FROM_JSON(trajectory, teb_autosize);
                FROM_JSON(trajectory, dt_ref);
                FROM_JSON(trajectory, dt_hysteresis);
                FROM_JSON(trajectory, min_samples);
                FROM_JSON(trajectory, max_samples);
                FROM_JSON(trajectory, global_plan_overwrite_orientation);
                FROM_JSON(trajectory, allow_init_with_backwards_motion);
                FROM_JSON(trajectory, global_plan_viapoint_sep);
                FROM_JSON(trajectory, via_points_ordered);
                FROM_JSON(trajectory, max_global_plan_lookahead_dist);
                FROM_JSON(trajectory, exact_arc_length);
                FROM_JSON(trajectory, force_reinit_new_goal_dist);
                FROM_JSON(trajectory, feasibility_check_no_poses);
                FROM_JSON(trajectory, publish_feedback);

                // Robot
                FROM_JSON(robot, max_vel_x);
                FROM_JSON(robot, max_vel_x_backwards);
                FROM_JSON(robot, max_vel_y);
                FROM_JSON(robot, max_vel_theta);
                FROM_JSON(robot, acc_lim_x);
                FROM_JSON(robot, acc_lim_y);
                FROM_JSON(robot, acc_lim_theta);
                FROM_JSON(robot, min_turning_radius);
                FROM_JSON(robot, wheelbase);
                FROM_JSON(robot, cmd_angle_instead_rotvel);
                FROM_JSON(robot, is_footprint_dynamic);

                // Goal Tolerance
                FROM_JSON(goal_tolerance, xy_goal_tolerance);
                FROM_JSON(goal_tolerance, yaw_goal_tolerance);
                FROM_JSON(goal_tolerance, free_goal_vel);

                // Obstacles

                FROM_JSON(obstacles, min_obstacle_dist);
                FROM_JSON(obstacles, inflation_dist);
                FROM_JSON(obstacles, dynamic_obstacle_inflation_dist);
                FROM_JSON(obstacles, include_dynamic_obstacles);
                FROM_JSON(obstacles, include_costmap_obstacles);
                FROM_JSON(obstacles, costmap_obstacles_behind_robot_dist);
                FROM_JSON(obstacles, obstacle_poses_affected);
                FROM_JSON(obstacles, legacy_obstacle_association);
                FROM_JSON(obstacles, obstacle_association_force_inclusion_factor);
                FROM_JSON(obstacles, obstacle_association_cutoff_factor);
                FROM_JSON(obstacles, costmap_converter_plugin);
                FROM_JSON(obstacles, costmap_converter_spin_thread);
                FROM_JSON(obstacles, costmap_converter_rate);

                // Optimization
                FROM_JSON(optim, no_inner_iterations);
                FROM_JSON(optim, no_outer_iterations);
                FROM_JSON(optim, optimization_activate);
                FROM_JSON(optim, optimization_verbose);
                FROM_JSON(optim, penalty_epsilon );
                FROM_JSON(optim, weight_max_vel_x);
                FROM_JSON(optim, weight_max_vel_y);
                FROM_JSON(optim, weight_max_vel_theta);
                FROM_JSON(optim, weight_acc_lim_x);
                FROM_JSON(optim, weight_acc_lim_y);
                FROM_JSON(optim, weight_acc_lim_theta);
                FROM_JSON(optim, weight_kinematics_nh);
                FROM_JSON(optim, weight_kinematics_forward_drive);
                FROM_JSON(optim, weight_kinematics_turning_radius);
                FROM_JSON(optim, weight_optimaltime);
                FROM_JSON(optim, weight_obstacle);
                FROM_JSON(optim, weight_inflation);
                FROM_JSON(optim, weight_dynamic_obstacle);
                FROM_JSON(optim, weight_dynamic_obstacle_inflation);
                FROM_JSON(optim, weight_viapoint);
                FROM_JSON(optim, weight_prefer_rotdir);
                FROM_JSON(optim, weight_adapt_factor);

                // Homotopy Class Planner
                FROM_JSON(hcp, enable_homotopy_class_planning);
                FROM_JSON(hcp, enable_multithreading);
                FROM_JSON(hcp, simple_exploration);
                FROM_JSON(hcp, max_number_classes);
                FROM_JSON(hcp, selection_cost_hysteresis);
                FROM_JSON(hcp, selection_prefer_initial_plan);
                FROM_JSON(hcp, selection_obst_cost_scale);
                FROM_JSON(hcp, selection_viapoint_cost_scale);
                FROM_JSON(hcp, selection_alternative_time_cost);
                FROM_JSON(hcp, obstacle_keypoint_offset);
                FROM_JSON(hcp, obstacle_heading_threshold);
                FROM_JSON(hcp, roadmap_graph_no_samples);
                FROM_JSON(hcp, roadmap_graph_area_width); // [m]
                FROM_JSON(hcp, roadmap_graph_area_length_scale);
                FROM_JSON(hcp, h_signature_prescaler);
                FROM_JSON(hcp, h_signature_threshold);
                FROM_JSON(hcp, viapoints_all_candidates);
                FROM_JSON(hcp, visualize_hc_graph);
                FROM_JSON(hcp, visualize_with_time_as_z_axis_scale);

                // Recovery
                FROM_JSON(recovery, shrink_horizon_backup);
                FROM_JSON(recovery, shrink_horizon_min_duration);
                FROM_JSON(recovery, oscillation_recovery);
                FROM_JSON(recovery, oscillation_v_eps);
                FROM_JSON(recovery, oscillation_omega_eps);
                FROM_JSON(recovery, oscillation_recovery_min_duration);
                FROM_JSON(recovery, oscillation_filter_duration);
            }
        }
    }
}
