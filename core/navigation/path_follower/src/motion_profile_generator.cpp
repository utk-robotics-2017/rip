#include "path_follower/motion_profile_generator.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            MotionProfile MotionProfileGenerator::generateProfile(const MotionProfileConstraints& constraints, const MotionProfileGoal& goal, const MotionState& previous)
            {
                units::Distance delta_position = goal.position() - previous.position();
                if(delta_position < 0.0 || (delta_position == 0.0 && previous.velocity() < 0.0))
                {
                    // For simplicity, we always assume the goal requires positive movement. If negative, we flip to solve, then
                    // flip the solution.
                    return generateFlippedProfile(constraints, goal, previous);
                }

                // Invariant from this point on: delta_pos >= 0.0
                // Clamp the start state to be valid.
                MotionState start_state(previous.t(), previous.position(), units::signum(previous.velocity()) * units::min(units::abs(previous.velocity()), constraints.maxVelocity()), units::signum(previous.acceleration()) * units::min(units::abs(previous.acceleration()), constraints.maxAcceleration()));
                MotionProfile profile;
                profile.reset(start_state);

                // If our velocity is headed away from the goal, the first thing we need to do is to stop.
                if(start_state.velocity() < 0.0 && delta_position > 0.0)
                {
                    const units::Time stopping_time = units::abs(start_state.velocity() / constraints.maxAcceleration());
                    profile.appendControl(constraints.maxAcceleration(), stopping_time);
                    start_state = profile.endState();
                    delta_position = goal.position() - start_state.position();
                }

                // Invariant from this point on: start_state.vel() >= 0.0
                const units::Units<2, -2, 0, 0, 0, 0> min_abs_vel_at_goal_sqr = units::pow<2>(start_state.velocity()) - 2.0 * constraints.maxAcceleration() * delta_position;
                const units::Velocity min_abs_vel_at_goal = units::sqrt(units::abs(min_abs_vel_at_goal_sqr));
                const units::Velocity max_abs_vel_at_goal = units::sqrt(units::pow<2>(start_state.velocity()) + 2.0 * constraints.maxAcceleration() * delta_position);
                units::Velocity goal_vel = goal.maxVelocity();
                units::Acceleration max_acc = constraints.maxAcceleration();

                if(min_abs_vel_at_goal_sqr > 0.0 && min_abs_vel_at_goal > (goal.maxVelocity() + goal.velocityTolerance()))
                {
                    // Overshoot is unavoidable with the current constraints. Look at completion_behavior to see what we should
                    // do.
                    if(goal.completionBehavior() == MotionProfileGoal::CompletionBehavior::kViolateMaxVel)
                    {
                        // Adjust the goal velocity.
                        goal_vel = min_abs_vel_at_goal;
                    }
                    else if(goal.completionBehavior() == MotionProfileGoal::CompletionBehavior::kViolateMaxAccel)
                    {
                        if(units::abs(delta_position) < goal.positionTolerance())
                        {
                            // Special case: We are at the goal but moving too fast. This requires 'infinite' acceleration,
                            // which will result in NaNs below, so we can return the profile immediately.
                            profile.appendSegment(MotionSegment(MotionState(profile.endTime(), profile.endPosition(), profile.endState().velocity(), -std::numeric_limits<double>::infinity()), MotionState(profile.endTime(), profile.endPosition(), goal_vel, -std::numeric_limits<double>::infinity())));
                            profile.consolidate();
                            return profile;
                        }
                        // Adjust the max acceleration.
                        max_acc = units::abs(goal_vel * goal_vel - units::pow<2>(start_state.velocity())) / (2.0 * delta_position);
                    }
                    else
                    {
                        // We are going to overshoot the goal, so the first thing we need to do is come to a stop.
                        const units::Time stopping_time = units::abs(start_state.velocity() / constraints.maxAcceleration());
                        profile.appendControl(-constraints.maxAcceleration(), stopping_time);
                        // Now we need to travel backwards, so generate a flipped profile.
                        profile.appendProfile(generateFlippedProfile(constraints, goal, profile.endState()));
                        profile.consolidate();
                        return profile;
                    }
                }
                goal_vel = units::min(goal_vel, max_abs_vel_at_goal);
                // Invariant from this point forward: We can achieve goal_vel at goal_state.pos exactly using no more than +/-
                // max_acc.

                // What is the maximum velocity we can reach (Vmax)? This is the intersection of two curves: one accelerating
                // towards the goal from profile.finalState(), the other coming from the goal at max vel (in reverse). If Vmax
                // is greater than constraints.max_abs_vel, we will clamp and cruise.
                // Solve the following three equations to find Vmax (by substitution):
                // Vmax^2 = Vstart^2 + 2*a*d_accel
                // Vgoal^2 = Vmax^2 - 2*a*d_decel
                // delta_pos = d_accel + d_decel
                const units::Velocity v_max = units::min(constraints.maxVelocity(), units::sqrt((units::pow<2>(start_state.velocity()) + goal_vel * goal_vel) / 2.0 + delta_position * max_acc));

                // Accelerate to v_max
                if(v_max > start_state.velocity())
                {
                    const units::Time accel_time = (v_max - start_state.velocity()) / max_acc;
                    profile.appendControl(max_acc, accel_time);
                    start_state = profile.endState();
                }
                // Figure out how much distance will be covered during deceleration.
                const units::Distance distance_decel = units::max(units::Distance(0.0), (units::pow<2>(start_state.velocity()) - goal_vel * goal_vel) / (2.0 * constraints.maxAcceleration()));
                const units::Distance distance_cruise = units::max(units::Distance(0.0), goal.position() - start_state.position() - distance_decel);
                // Cruise at constant velocity.
                if(distance_cruise > 0.0)
                {
                    const units::Time cruise_time = distance_cruise / start_state.velocity();
                    profile.appendControl(0.0, cruise_time);
                    start_state = profile.endState();
                }
                // Decelerate to goal velocity.
                if(distance_decel > 0.0)
                {
                    const units::Time decel_time = (start_state.velocity() - goal_vel) / max_acc;
                    profile.appendControl(-max_acc, decel_time);
                }

                profile.consolidate();
                return profile;
            }

            MotionProfile MotionProfileGenerator::generateFlippedProfile(const MotionProfileConstraints& constraints, const MotionProfileGoal& goal, const MotionState& previous)
            {
                MotionProfile profile = generateProfile(constraints, goal.flipped(), previous.flipped());
                for(MotionSegment& segment : profile.segments())
                {
                    segment.setStart(segment.start().flipped());
                    segment.setEnd(segment.end().flipped());
                }
                return profile;
            }
        }
    }
}