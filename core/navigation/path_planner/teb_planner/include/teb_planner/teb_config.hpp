#ifndef TEB_CONFIG_HPP
#define TEB_CONFIG_HPP

#include <units/units.hpp>

namespace rip
{
    namespace navigation
    {
        class TebConfig
        {
        public:
            /**
             * Trajectory related parameters
             */
            struct Trajectory
            {
                /**
                 * Enable automatic resizing of the trajectory w.r.t. the
                 * temporal resolution (recommended)
                 */
                double autosize;

                /**
                 * Desired temporal resolution of the trajectory (should be in
                 * the magnitude of the underlying control rate)
                 */
                double dt_ref;

                /**
                 * Hysteresis for automatic resizing depending on the current
                 * temporal resolution (dt): usually 10%  of dt_ref
                 */
                double dt_hysteresis;

                /**
                 * Minimum number of samples (should always be greater than 2)
                 */
                int min_samples;

                /**
                 * Maximum number of samples.
                 *
                 * @warning If too small the discretization/resolution might not
                 * be sufficient for the given robot model or obstacle avoidance
                 * might not work anymore
                 */
                int max_samples;

                /**
                 * Overwrite the orientation of the local subgoals provided
                 * by the global planner
                 */
                bool global_plan_overwrite_orientation;

                /**
                 * If true, the underlying trajectories might be initialized
                 * with backwards motion in case the goal is behind the start
                 * within the local costmap (this is only recommended if the
                 * robot is equipped with rear sensors)
                 */
                bool allow_init_with_backwards_motion;

                /**
                 * Minimum separation between two consecutive waypoints from
                 * the global plan (if negative: disabled)
                 */
                units::Distance global_plan_waypoint_separation;


                /**
                 * If true, the planner adheres to the order of the waypoints
                 * in the storage container
                 */
                bool waypoints_ordered;

                /**
                 * Specify the maximum length (cumulative Euclidean distances)
                 * of the subset of the global plan taken into account for
                 * optimization [if <= 0: disabled; The length is also
                 * bounded by the local costmap size]
                 */
                units::Distance max_global_plan_lookahead_distance;

                /**
                 * If true, the planner uses the exact arc length in velocity,
                 * acceleration, and turning rate computations [-> increased
                 * cpu time], otherwise the euclidean approximation is used
                 */
                bool exact_arc_length;

                /**
                 * Reinitialize the trajectory if the previous goal is updated
                 * with new goal with a separation more than the given value
                 */
                units::Distance force_reinit_new_goal_distance;

                /**
                 * Specify up to which pose on the predicted plan the feasibility
                 * should be checked each sampling interval
                 */
                int feasibility_check_no_poses;
            } trajectory;

            /**
             * Robot related parameters
             */
            struct Robot
            {
                /**
                 * Maximum translational velocity of the robot
                 */
                units::Velocity max_velocity_x;

                /**
                 * Maximum translational velocity of the robot for driving
                 * backwards
                 */
                units::Velocity max_velocity_x_backwards;

                /**
                 * Maximum strafing velocity of the robot (should be zero for
                 * non-holonomic robots)
                 * @note This should be zero for non-holonomic robots
                 */
                units::Velocity max_velocity_y;

                /**
                 * Maximum angular velocity of the robot
                 */
                units::AngularVelocity max_velocity_theta;

                /**
                 * Maximum translational acceleration of the robot
                 */
                units::Acceleration acceleration_limit_x;

                /**
                 * Maximum strafing acceleration of the robot
                 */
                units::Acceleration acceleration_limit_y;

                /**
                 * Maximum angular acceleration of the robot
                 */
                units::AngularAcceleration acceleration_limit_theta;

                /**
                 * Minimum turning radius for Ackerman steering (car-like)
                 *
                 * @note For differential drive robots this should be zero
                 */
                units::Distance min_turning_radius;

                /**
                 * The distance between the drive shaft and the steering axle
                 *
                 * @note Only required for Ackerman steering (car-like)
                 */
                units::Distance wheelbase;

                /**
                 * If true, updated the footprint before checking trajectory
                 * feasibility
                 */
                bool is_footprint_dynamic;
            } robot;

            /**
             * Parameters related to the tolerances on the final pose
             */
            struct GoalTolerance
            {
                /**
                 * Allowed final orientation error
                 */
                units::Angle yaw_goal_tolerance;

                /**
                 * Allowed final euclidean distance to the goal position
                 */
                units::Distance xy_goal_tolerance;

                /**
                 * Allows the robot's velocity to be non-zero (usually
                 * max_velocity) upon reaching the goal pose
                 */
                bool free_goal_velocity;
            } goal_tolerance;

            /**
             * Obstacle related parameters
             */
            struct Obstacles
            {
                /**
                 * Minimum desired separation from obstacles
                 */
                units::Distance min_obstacle_distance;

                /**
                 * Buffer zone around obstacles with non-zero penalty cost
                 * (should be larger than min_obstacle_distance in order to
                 * take effect)
                 */
                units::Distance inflation_distance;

                /**
                 * Buffer zone around predicted locations of dynamic
                 * obstacles with non-zero penalty costs (should be larger
                 * min_obstacle_distance in order to take effect)
                 */
                units::Distance dynamic_obstacle_inflation_distance;

                /**
                 * Specify whether the movement of dynamic obstacles should be
                 * predicted by a constant velocity model.
                 *
                 * If false all obstacles are considered to be static.
                 */
                bool include_dynamic_obstacles;

                /**
                 * Specify whether the obstacles in the costmap should be taken
                 * into account directly
                 */
                bool include_costmap_obstacles;

                /**
                 * Limit the occupied local costmap obstacles taken into account
                 * for planning behind the robot
                 */
                units::Distance costmap_obstacles_behind_robot_distance;

                /**
                 * The obstacles position is attached to the closest pose on
                 * the trajectory to reduce computational effort, but take a
                 * number of neighbors into account as well
                 */
                int obstacles_poses_affected;

                /**
                 * The obstacle association technique tries to connect only
                 * relevant obstacles with the discretized trajectory during
                 * optimization, all obstacles within a specified distance are
                 * forced to be included (as a multiple of min_obstacle_distance),
                 *  e.g. choose 2.0 in order to consider obstacles within a radius
                 *  of 2.0*min_obstacle_dist.
                 */
                double obstacle_association_force_inclusion_factor;

                /**
                 * See obstacle_association_force_inclusion_factor, but beyond a
                 * multiple of [value]*min_obstacle_dist all obstacles are ignored
                 * during optimization.
                 */
                double obstacle_association_cutoff_factor;
            } obstacles;

            /**
             * Optimization related parameters
             */
            struct Optimization
            {
                /**
                 * Number of solver iterations called in each outer loop
                 * iterations
                 */
                int num_inner_iterations;

                /**
                 * Each outer loop iteration automatically resizes the
                 * trajectory and invokes the internal optimizer with
                 * num_inner_iterations
                 */
                int num_outer_iterations;

                /**
                 * Add a small safety margin to penalty functions for
                 * hard-constraint approximations
                 */
                double penalty_epsilon;

                /**
                 * Optimization weight for satisfying the maximum allowed
                 * translational velocity
                 */
                double max_velocity_x_weight;

                /**
                 * Optimization weight for satisfying the maximum allowed
                 * strafing velocity
                 */
                double max_velocity_y_weight;

                /**
                 * Optimization weight for satisfying the maximum allowed
                 * angular velocity
                 */
                double max_velocity_theta_weight;

                /**
                 * Optimization weight for satisfying the maximum allowed
                 * translational acceleration
                 */
                double acceleration_limit_x_weight;

                /**
                 * Optimization weight for satisfying the maximum allowed
                 * strafing acceleration
                 */
                double acceleration_limit_y_weight;

                /**
                 * Optimization weight for satisfying the maximum allowed
                 * angular acceleration
                 */
                double acceleration_limit_theta_weight;

                /**
                 * Optimization weight for satisfying the non-holonomic
                 * kinematics
                 */
                double kinematics_nh_weight;

                /**
                 * Optimization weight for forcing the robot to choose
                 * only forward directions
                 *
                 * @note Differential drive only
                 */
                double kinematics_forward_drive_weight;

                /**
                 * Optimization weight for enforcing a minimum turning radius
                 *
                 * @note Ackerman steering only
                 */
                double kinematics_turning_radius_weight;

                /**
                 * Optimization weight for contracting the trajectory w.r.t.
                 * transition time
                 */
                double optimal_time_weight;

                /**
                 * Optimization weight for satisfying a minimum separation
                 * from obstacles
                 */
                double obstacle_weight;

                /**
                 * Optimization weight for the inflation penalty (should be
                 * small)
                 */
                double inflation_weight;

                /**
                 * Optimization weight for satisfying minimum separation
                 * from dynamic obstacles
                 */
                double dynamic_obstacle_weight;

                /**
                 * Optimization weight for the inflation penalty of
                 * dynamic obstacles (should be small)
                 */
                double dynamic_obstacle_inflation_weight;

                /**
                 * Optimization weight for the inflation penalty of dynamic
                 * obstacles (should be small)
                 */
                double waypoint_weight;

                /**
                 * Some special weights (currently 'obstacle_weight') are
                 * repeatedly scaled by this factor in each out TEB iteration
                 */
                double weight_adapt_factor;
            } optimization;
        };

        void from_json(const nlohmann::json& j, TebConfig& c);

        void to_json(nlohmann::json& j, const TebConfig& c);
    }
}

#endif // TEB_CONFIG_HPP
