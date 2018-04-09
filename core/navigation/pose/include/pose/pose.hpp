#ifndef POSE_HPP
#define POSE_HPP

#include <units/units.hpp>

namespace rip
{
    namespace navigation
    {
        namespace pose
        {
            struct Pose
            {
                units::Distance x;
                units::Distance y;
                units::Distance z;
                units::Angle yaw;
                units::Angle pitch;
                units::Angle roll;
                units::Velocity d_x;
                units::Velocity d_y;
                units::Velocity d_z;
                units::AngularVelocity d_yaw;
                units::AngularVelocity d_pitch;
                units::AngularVelocity d_roll;
                units::Acceleration dd_x;
                units::Acceleration dd_y;
                units::Acceleration dd_z;
            };
        }
    }
}

#endif //POSE_HPP
