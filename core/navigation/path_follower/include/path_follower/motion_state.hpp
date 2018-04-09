#ifndef MOTION_STATE_HPP
#define MOTION_STATE_HPP

#include <ostream>

#include <units/units.hpp>

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            class MotionState
            {
            public:
                MotionState(const units::Time& t, const units::Distance& position, const units::Velocity& velocity, const units::Acceleration& acceleration);

                MotionState(const MotionState& state);

                units::Time t() const;
                units::Distance position() const;
                units::Velocity velocity() const;
                units::Acceleration acceleration() const;

                MotionState extrapolate(const units::Time& t) const;
                MotionState extrapolate(const units::Time& t, const units::Acceleration& acceleration) const;

                units::Time nextTimeAtPosition(const units::Distance& position);

                std::string toString() const;
                friend std::ostream& operator<<(std::ostream& os, const MotionState& profile);

                bool operator==(const MotionState& rhs);
                bool operator!=(const MotionState& rhs);
                bool equals(const MotionState& rhs, const double epsilon);
                bool coincident(const MotionState& rhs);
                bool coincident(const MotionState& rhs, const double epsilon);

                MotionState flipped() const;

                static MotionState k_invalid_state;

            private:
                units::Time m_t;
                units::Distance m_position;
                units::Velocity m_velocity;
                units::Acceleration m_acceleration;
            };

            std::ostream& operator<<(std::ostream& os, const MotionState& profile);
        }
    }
}

#endif //MOTION_STATE_HPP
