#include "path_follower/motion_profile_goal.hpp"

#include "path_follower/epsilon.hpp"

#include <fmt/format.h>

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            MotionProfileGoal::MotionProfileGoal()
                    : m_position(0.0)
                    , m_max_abs_velocity(0.0)
                    , m_completion_behavior(CompletionBehavior::kOvershoot)
                    , m_position_tolerance(1e-3 * units::m)
                    , m_velocity_tolerance(1e-2 * units::m / units::s)
            {}

            MotionProfileGoal::MotionProfileGoal(const units::Distance& position)
                    : m_position(position)
                    , m_max_abs_velocity(0.0)
                    , m_completion_behavior(CompletionBehavior::kOvershoot)
                    , m_position_tolerance(1e-3 * units::m)
                    , m_velocity_tolerance(1e-2 * units::m / units::s)
            {
                sanityCheck();
            }

            MotionProfileGoal::MotionProfileGoal(const units::Distance& position, const units::Velocity& max_abs_vel)
                    : m_position(position)
                    , m_max_abs_velocity(max_abs_vel)
                    , m_completion_behavior(CompletionBehavior::kOvershoot)
                    , m_position_tolerance(1e-3 * units::m)
                    , m_velocity_tolerance(1e-2 * units::m / units::s)
            {
                sanityCheck();
            }

            MotionProfileGoal::MotionProfileGoal(const units::Distance& position, const units::Velocity& max_abs_vel, CompletionBehavior completion_behavior)
                    : m_position(position)
                    , m_max_abs_velocity(max_abs_vel)
                    , m_completion_behavior(completion_behavior)
                    , m_position_tolerance(1e-3 * units::m)
                    , m_velocity_tolerance(1e-2 * units::m / units::s)
            {
                sanityCheck();
            }

            MotionProfileGoal::MotionProfileGoal(const units::Distance& position, const units::Velocity& max_abs_vel, CompletionBehavior completion_behavior, const units::Distance& position_tolerance, const units::Velocity& velocity_tolerance)
                    : m_position(position)
                    , m_max_abs_velocity(max_abs_vel)
                    , m_completion_behavior(completion_behavior)
                    , m_position_tolerance(position_tolerance)
                    , m_velocity_tolerance(velocity_tolerance)
            {
                sanityCheck();
            }

            MotionProfileGoal::MotionProfileGoal(const MotionProfileGoal& other)
                    : m_position(other.m_position)
                    , m_max_abs_velocity(other.m_max_abs_velocity)
                    , m_completion_behavior(other.m_completion_behavior)
                    , m_position_tolerance(other.m_position_tolerance)
                    , m_velocity_tolerance(other.m_velocity_tolerance)
            {}

            MotionProfileGoal MotionProfileGoal::flipped() const
            {
                return MotionProfileGoal(-m_position, m_max_abs_velocity, m_completion_behavior, m_position_tolerance, m_velocity_tolerance);
            }

            units::Distance MotionProfileGoal::position() const
            {
                return m_position;
            }

            units::Velocity MotionProfileGoal::maxVelocity() const
            {
                return m_max_abs_velocity;
            }

            units::Distance MotionProfileGoal::positionTolerance() const
            {
                return m_position_tolerance;
            }

            units::Velocity MotionProfileGoal::velocityTolerance() const
            {
                return m_velocity_tolerance;
            }

            MotionProfileGoal::CompletionBehavior MotionProfileGoal::completionBehavior() const
            {
                return m_completion_behavior;
            }

            bool MotionProfileGoal::atGoalState(const MotionState& state) const
            {
                return atGoalPosition(state.position()) && (units::abs(state.velocity()) < (m_max_abs_velocity < m_velocity_tolerance) ||
                       m_completion_behavior == CompletionBehavior::kViolateMaxVel);
            }

            bool MotionProfileGoal::atGoalPosition(const units::Distance& position) const
            {
                return epsilonEquals(position, m_position, m_position_tolerance);
            }

            std::string MotionProfileGoal::toString() const
            {
                return fmt::format("pos: {} (+/- {}) in, max vel: {} (+/- {}) in/s, completion behavior: {}", m_position.to(units::in), m_position_tolerance.to(units::in), m_max_abs_velocity.to(units::in / units::s), m_velocity_tolerance.to(units::in / units::s), toString(m_completion_behavior));
            }

            bool MotionProfileGoal::operator==(const MotionProfileGoal& rhs)
            {
                return m_position == rhs.m_position && m_position_tolerance == rhs.m_position_tolerance && m_max_abs_velocity == rhs.m_max_abs_velocity && m_velocity_tolerance == rhs.m_velocity_tolerance && m_completion_behavior == rhs.m_completion_behavior;
            }

            bool MotionProfileGoal::operator!=(const MotionProfileGoal& rhs)
            {
                return !(*this == rhs);
            }

            void MotionProfileGoal::sanityCheck()
            {
                if(m_max_abs_velocity > m_velocity_tolerance && m_completion_behavior == CompletionBehavior::kOvershoot)
                {
                    m_completion_behavior = CompletionBehavior::kViolateMaxAccel;
                }
            }

            std::string MotionProfileGoal::toString(CompletionBehavior completion_behavior) const
            {
                switch(completion_behavior)
                {
                    case CompletionBehavior::kOvershoot:
                        return "overshoot";
                    case CompletionBehavior::kViolateMaxVel:
                        return "violate max vel";
                    case CompletionBehavior::kViolateMaxAccel:
                        return "violate max accel";
                    default:
                        assert(false);
                }
            }

            std::ostream& operator<<(std::ostream& os, const MotionProfileGoal& goal)
            {
                os << goal.toString() << std::endl;
                return os;
            }
        }
    }
}