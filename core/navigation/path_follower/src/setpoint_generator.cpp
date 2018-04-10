#include "path_follower/setpoint_generator.hpp"
#include "path_follower/motion_profile_generator.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            Setpoint::Setpoint(const MotionState& state, bool final)
                : m_state(state)
                , m_final(final)
            {
            }

            MotionState Setpoint::state() const
            {
                return m_state;
            }

            void Setpoint::setState(const MotionState& state)
            {
                m_state = state;
            }

            bool Setpoint::final() const
            {
                return m_final;
            }

            void Setpoint::setFinal(bool final)
            {
                m_final = final;
            }

            SetpointGenerator::SetpointGenerator()
                    : m_profile(nonstd::nullopt), m_goal(nonstd::nullopt), m_constraints(nonstd::nullopt)
            {
            }

            void SetpointGenerator::reset()
            {
                m_profile = nonstd::nullopt;
                m_goal = nonstd::nullopt;
                m_constraints = nonstd::nullopt;
            }

            Setpoint SetpointGenerator::setpoint(const MotionProfileConstraints& constraints, const MotionProfileGoal& goal, const MotionState& previous, const units::Time& t)
            {
                bool regenerate = !m_constraints || m_constraints.value() != constraints || !m_goal || m_goal.value() != goal || !m_profile;

                if(!regenerate && !(m_profile.value().empty()))
                {
                    nonstd::optional<MotionState> expected_state = m_profile.value().stateByTime(previous.t());
                    regenerate = !expected_state || (*expected_state) != previous;
                }

                if(regenerate)
                {
                    // Regenerate the profile, as our current profile does not satisfy the inputs.
                    m_constraints = constraints;
                    m_goal = goal;
                    m_profile = MotionProfileGenerator::generateProfile(constraints, goal, previous);
                }
            }

            nonstd::optional<MotionProfile> SetpointGenerator::profile() const
            {
                return m_profile;
            }
        }
    }
}