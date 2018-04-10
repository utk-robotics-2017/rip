#ifndef FILTER_COMMON_HPP
#define FILTER_COMMON_HPP

namespace rip
{
    namespace navigation
    {
        namespace pose
        {
            /**
             * Enumeration that defines the state vector
             */
            enum class StateMembers
            {
                kX = 0,
                kY,
                kZ,
                kRoll,
                kPitch,
                kYaw,
                kVx,
                kVy,
                kVz,
                kVroll,
                kVpitch,
                kVyaw,
                kAx,
                kAy,
                kAz
            };

            /**
             * Enumeration that defines the control vector
             */
            enum class ControlMembers
            {
                kVx,
                kVy,
                kVz,
                kVroll,
                kVpitch,
                kVyaw
            };

            /**
             * Global constants that define our state
             * vector size and offsets to groups of values
             * within that state.
             */
            const int kStateSize = 15;
            const int kPositionOffset = static_cast<int>(StateMembers::kX);
            const int kOrientationOffset = static_cast<int>(StateMembers::kRoll);
            const int kVelocityOffset = static_cast<int>(StateMembers::kVx);
            const int kAngularVelocityOffset = static_cast<int>(StateMembers::kVroll);
            const int kAccelerationOffset = static_cast<int>(StateMembers::kAx);
        }
    }
}

#endif //FILTER_COMMON_HPP
