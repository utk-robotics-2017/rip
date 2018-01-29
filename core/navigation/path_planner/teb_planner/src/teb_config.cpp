#include <teb_planner/teb_config.hpp>

namespace rip
{
    namespace navigation
    {
        void from_json(const nlohmann::json& j, TebConfig& c)
        {
            // Trajectory
            c.trajectory.autosize = j["autosize"];
            c.trajectory.dt_ref = j["dt_ref"];
            c.trajectory.dt_hysteresis = j["dt_hysteresis"];
            c.trajectory.min_samples = j["min_samples"];
            c.trajectory.max_samples = j["max_samples"];
            c.trajectory.global_plan_overwrite_orientation = j["global_plan_overwrite_orientation"];
            c.trajectory.allow_init_with_backwards_motion = j["allow_init_with_backwards_motion"];
            c.trajectory.global_plan_waypoint_separation = j["global_plan_waypoint_separation"];
            c.trajectory.waypoints_ordered = j["waypoint_ordered"];
            c.trajectory.max_global_plan_lookahead_distance = j["max_global_plan_lookahead_distance"];
            c.trajectory.exact_arc_length = j["exact_arc_length"];
            c.trajectory.force_reinit_new_goal_distance = j["force_reinit_new_goal_distance"];
            c.trajectory.feasibility_check_no_poses = j["feasibility_check_no_poses"];

            // Robot
            c.robot.max_velocity_x = j["max_velocity_x"];
            c.robot.max_velocity_x_backwards = j["max_velocity_x_backwards"];
            c.robot.max_velocity_y = j["max_velocity_y"];
            c.robot.max_velocity_theta = j["max_velocity_theta"];
            c.robot.acceleration_limit_x = j["acceleration_limit_x"];
            c.robot.acceleration_limit_y = j["acceleration_limit_y"];
            c.robot.acceleration_limit_theta = j["acceleration_limit_theta"];
            c.robot.min_turning_radius = j["min_turning_radius"];
            c.robot.wheelbase = j["wheelbase"];

            // Goal Tolerance
            c.goal_tolerance.yaw_goal_tolerance = j["yaw_goal_tolerance"];
            c.goal_tolerance.xy_goal_tolerance = j["xy_goal_tolerance"];
            c.goal_tolerance.free_goal_velocity = j["free_goal_velocity"];

            // Obstacles
            c.obstacles.min_obstacle_distance = j["min_obstacle_distance"];
            c.obstacles.inflation_distance = j["inflation_distance"];
            c.obstacles.dynamic_obstacle_inflation_distance = j["dynamic_obstacle_inflation_distance"];
            c.obstacles.include_dynamic_obstacles = j["include_dynamic_obstacles"];
            c.obstacles.obstacles_poses_affected = j["obstacles_poses_affected"];
            c.obstacles.obstacle_association_force_inclusion_factor = j["obstacle_association_force_inclusion_factor"];
            c.obstacles.obstacle_association_cutoff_factor = j["obstacle_association_cutoff_factor"];

            // Optimization
            c.optimization.num_inner_iterations = j["num_inner_iterations"];
            c.optimization.num_outer_iterations = j["num_outer_iterations"];
            c.optimization.penalty_epsilon = j["penalty_epsilon"];
            c.optimization.max_velocity_x_weight = j["max_velocity_x_weight"];
            c.optimization.max_velocity_y_weight = j["max_velocity_y_weight"];
            c.optimization.max_velocity_theta_weight = j["max_velocity_theta_weight"];
            c.optimization.acceleration_limit_x_weight = j["acceleration_limit_x_weight"];
            c.optimization.acceleration_limit_y_weight = j["acceleration_limit_y_weight"];
            c.optimization.acceleration_limit_theta_weight = j["acceleration_limit_theta_weight"];
            c.optimization.kinematics_nh_weight = j["kinematics_nh_weight"];
            c.optimization.kinematics_forward_drive_weight = j["kinematics_forward_drive_weight"];
            c.optimization.kinematics_turning_radius_weight = j["kinematics_turning_radius_weight"];
            c.optimization.optimal_time_weight = j["optimal_time_weight"];
            c.optimization.obstacle_weight = j["obstacle_weight"];
            c.optimization.inflation_weight = j["inflation_weight"];
            c.optimization.dynamic_obstacle_weight = j["dynamic_obstacle_weight"];
            c.optimization.dynamic_obstacle_inflation_weight = j["dynamic_obstacle_inflation_weight"];
            c.optimization.waypoint_weight = j["waypoint_weight"];
            c.optimization.weight_adapt_factor = j["weight_adapt_factor"];
        }

        void to_json(nlohmann::json& j, const TebConfig& c)
        {
            j = nlohmann::json
            {
                // Trajectory
                {"autosize", c.trajectory.autosize},
                {"dt_ref", c.trajectory.dt_ref},
                {"dt_hysteresis", c.trajectory.dt_hysteresis},
                {"min_samples", c.trajectory.min_samples},
                {"max_samples", c.trajectory.max_samples},
                {"global_plan_overwrite_orientation", c.trajectory.global_plan_overwrite_orientation},
                {"allow_init_with_backwards_motion", c.trajectory.allow_init_with_backwards_motion},
                {"global_plan_waypoint_separation", c.trajectory.global_plan_waypoint_separation},
                {"waypoint_ordered", c.trajectory.waypoints_ordered},
                {"max_global_plan_lookahead_distance", c.trajectory.max_global_plan_lookahead_distance},
                {"exact_arc_length", c.trajectory.exact_arc_length},
                {"force_reinit_new_goal_distance", c.trajectory.force_reinit_new_goal_distance},
                {"feasibility_check_no_poses", c.trajectory.feasibility_check_no_poses},
                // Robot
                {"max_velocity_x", c.robot.max_velocity_x},
                {"max_velocity_x_backwards", c.robot.max_velocity_x_backwards},
                {"max_velocity_y", c.robot.max_velocity_y},
                {"max_velocity_theta", c.robot.max_velocity_theta},
                {"acceleration_limit_x", c.robot.acceleration_limit_x},
                {"acceleration_limit_y", c.robot.acceleration_limit_y},
                {"acceleration_limit_theta", c.robot.acceleration_limit_theta},
                {"min_turning_radius", c.robot.min_turning_radius},
                {"wheelbase", c.robot.wheelbase},
                // Goal Tolerance
                {"yaw_goal_tolerance", c.goal_tolerance.yaw_goal_tolerance},
                {"xy_goal_tolerance", c.goal_tolerance.xy_goal_tolerance},
                {"free_goal_velocity", c.goal_tolerance.free_goal_velocity},
                // Obstacles
                {"min_obstacle_distance", c.obstacles.min_obstacle_distance},
                {"inflation_distance", c.obstacles.inflation_distance},
                {"dynamic_obstacle_inflation_distance", c.obstacles.dynamic_obstacle_inflation_distance},
                {"include_dynamic_obstacles", c.obstacles.include_dynamic_obstacles},
                {"obstacles_poses_affected", c.obstacles.obstacles_poses_affected},
                {"obstacle_association_force_inclusion_factor", c.obstacles.obstacle_association_force_inclusion_factor},
                {"obstacle_association_cutoff_factor", c.obstacles.obstacle_association_cutoff_factor},
                // Optimization
                {"num_inner_iterations", c.optimization.num_inner_iterations},
                {"num_outer_iterations", c.optimization.num_outer_iterations},
                {"penalty_epsilon", c.optimization.penalty_epsilon},
                {"max_velocity_x_weight", c.optimization.max_velocity_x_weight},
                {"max_velocity_y_weight", c.optimization.max_velocity_y_weight},
                {"max_velocity_theta_weight", c.optimization.max_velocity_theta_weight},
                {"acceleration_limit_x_weight", c.optimization.acceleration_limit_x_weight},
                {"acceleration_limit_y_weight", c.optimization.acceleration_limit_y_weight},
                {"acceleration_limit_theta_weight", c.optimization.acceleration_limit_theta_weight},
                {"kinematics_nh_weight", c.optimization.kinematics_nh_weight},
                {"kinematics_forward_drive_weight", c.optimization.kinematics_forward_drive_weight},
                {"kinematics_turning_radius_weight", c.optimization.kinematics_turning_radius_weight},
                {"optimal_time_weight", c.optimization.optimal_time_weight},
                {"obstacle_weight", c.optimization.obstacle_weight},
                {"inflation_weight", c.optimization.inflation_weight},
                {"dynamic_obstacle_weight", c.optimization.dynamic_obstacle_weight},
                {"dynamic_obstacle_inflation_weight", c.optimization.dynamic_obstacle_inflation_weight},
                {"waypoint_weight", c.optimization.waypoint_weight},
                {"weight_adapt_factor", c.optimization.weight_adapt_factor}
            };
        }
    }
}
