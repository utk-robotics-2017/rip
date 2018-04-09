#include "pose/imu.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pose
        {
            Imu::Imu()
            {}

            units::Time Imu::time() const
            {
                return m_time;
            }

            units::Angle Imu::yaw() const
            {
                return m_yaw;
            }

            units::Angle Imu::pitch() const
            {
                return m_pitch;
            }

            units::Angle Imu::roll() const
            {
                return m_roll;
            }

            units::AngularVelocity Imu::dYaw() const
            {
                return m_yaw_velocity;
            }

            units::AngularVelocity Imu::dPitch() const
            {
                return m_pitch_velocity;
            }

            units::AngularVelocity Imu::dRoll() const
            {
                return m_roll_velocity;
            }

            units::Acceleration Imu::ddX() const
            {
                return m_accel_x;
            }

            units::Acceleration Imu::ddY() const
            {
                return m_accel_y;
            }

            units::Acceleration Imu::ddZ() const
            {
                return m_accel_z;
            }
        }
    }
}
