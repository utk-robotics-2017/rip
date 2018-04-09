#ifndef IMU_HPP
#define IMU_HPP

#include <units/units.hpp>

namespace rip
{
    namespace navigation
    {
        namespace pose
        {
            class Imu
            {
            public:
                Imu();

                units::Time time() const;
                units::Angle yaw() const;
                units::Angle pitch() const;
                units::Angle roll() const;

                units::AngularVelocity dYaw() const;
                units::AngularVelocity dPitch() const;
                units::AngularVelocity dRoll() const;

                units::Acceleration ddX() const;
                units::Acceleration ddY() const;
                units::Acceleration ddZ() const;

            private:
                units::Time m_time;
                units::Angle m_yaw;
                units::Angle m_pitch;
                units::Angle m_roll;
                units::AngularVelocity m_yaw_velocity;
                units::AngularVelocity m_pitch_velocity;
                units::AngularVelocity m_roll_velocity;
                units::Acceleration m_accel_x;
                units::Acceleration m_accel_y;
                units::Acceleration m_accel_z;
            };
        }
    }
}

#endif //IMU_HPP
