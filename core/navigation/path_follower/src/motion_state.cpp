#include "path_follower/motion_state.hpp"
#include "path_follower/epsilon.hpp"

#include <fmt/format.h>

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            MotionState::MotionState(const units::Time& t, const units::Distance& position, const units::Velocity& velocity, const units::Acceleration& acceleration)
                    : m_t(t)
                    , m_position(position)
                    , m_velocity(velocity)
                    , m_acceleration(acceleration)
            {}

            MotionState::MotionState(const MotionState& state)
                    : m_t(state.m_t)
                    , m_position(state.m_position)
                    , m_velocity(state.m_velocity)
                    , m_acceleration(state.m_acceleration)
            {}

            units::Time MotionState::t() const
            {
                return m_t;
            }

            units::Distance MotionState::position() const
            {
                return m_position;
            }

            units::Velocity MotionState::velocity() const
            {
                m_velocity;
            }

            units::Acceleration MotionState::acceleration() const
            {
                return m_acceleration;
            }

            MotionState MotionState::extrapolate(const units::Time& t) const
            {
                return extrapolate(t, m_acceleration);
            }

            MotionState MotionState::extrapolate(const units::Time& t, const units::Acceleration& acceleration) const
            {
                const units::Time dt = t - m_t;
                return MotionState(t, m_position + m_velocity * dt + 0.5 * acceleration * dt * dt, m_velocity + acceleration * dt, acceleration);
            }

            units::Time MotionState::nextTimeAtPosition(const units::Distance& position)
            {
                if(epsilonEquals<units::Distance>(position, m_position, k_epsilon))
                {
                    // Already at position
                    return m_t;
                }

                if(epsilonEquals<units::Acceleration>(m_acceleration, 0.0, k_epsilon))
                {
                    // Zero acceleration case
                    const units::Distance delta_position = position - m_position;
                    if(!epsilonEquals<units::Velocity>(m_velocity, 0.0, k_epsilon) && units::signum(delta_position) == units::signum(m_velocity))
                    {
                        // Constant velocity heading towards position
                        return delta_position / m_velocity + m_t;
                    }
                    return std::nan("");
                }

                /**
                 * Solve the quadratic formula.
                 * ax^2 + bx + c == 0
                 * x = dt
                 * a = 0.5 * m_acceleration
                 * b = m_velocity
                 * c = m_position - position
                 */
                const units::Units<2,-2,0,0,0,0> disc = m_velocity * m_velocity - 2.0 * m_acceleration * (m_position - position);
                if(disc < 0.0)
                {
                    // Extrapolating this MotionState never reaches the desired position
                    return std::nan("");
                }
                const units::Velocity sqrt_disc = units::sqrt(disc);
                const units::Time max_dt = (-m_velocity + sqrt_disc) / m_acceleration;
                const units::Time min_dt = (-m_velocity - sqrt_disc) / m_acceleration;
                if(min_dt >= 0.0 && (max_dt < 0.0 || min_dt < max_dt))
                {
                    return m_t + max_dt;
                }
                if(max_dt >= 0.0)
                {
                    return m_t + max_dt;
                }
                // We only reach the desired position in the past
                return std::nan("");
            }

            std::string MotionState::toString() const
            {
                return fmt::format("(t={} s, pos={} in, vel={} in/s, acc={} in/s^2)", m_t.to(units::s), m_position.to(units::in), m_velocity.to(units::in / units::s), m_acceleration.to(units::in / units::s / units::s));
            }

            bool MotionState::operator==(const MotionState& rhs)
            {
                return equals(rhs, k_epsilon);
            }

            bool MotionState::operator!=(const MotionState& rhs)
            {
                return !equals(rhs, k_epsilon);
            }

            bool MotionState::equals(const MotionState& rhs, const double epsilon)
            {
                return coincident(rhs, epsilon) && epsilonEquals<units::Acceleration>(m_acceleration, rhs.m_acceleration, epsilon);
            }

            bool MotionState::coincident(const MotionState& rhs)
            {
                return coincident(rhs, k_epsilon);
            }

            bool MotionState::coincident(const MotionState& rhs, const double epsilon)
            {
                return epsilonEquals<units::Time>(m_t, rhs.m_t, epsilon) && epsilonEquals<units::Distance>(m_position, rhs.m_position, epsilon) && epsilonEquals<units::Velocity>(m_velocity, rhs.m_velocity, epsilon);
            }

            MotionState MotionState::flipped() const
            {
                return MotionState(m_t, -m_position, -m_velocity, -m_acceleration);
            }

            std::ostream& operator<<(std::ostream& os, const MotionState& state)
            {
                os << state.toString() << std::endl;
            }
        }
    }
}