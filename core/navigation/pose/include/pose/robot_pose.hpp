#ifndef ROBOT_POSE_HPP
#define ROBOT_POSE_HPP

#include <queue>

#include <units/units.hpp>
#include "pose/ukf.hpp"
#include "odometry.hpp"
#include "imu.hpp"
#include "pose.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pose
        {
            class RobotPose
            {
            public:
                RobotPose(const units::Distance& wheelbase, const double alpha=0.1, const double beta =0.1,const double kappa=0.0);

                void updateOdometry(const units::Time& time, const units::Distance& left_distance, const units::Distance& right_distance);
                void updateOdometry(const units::Time& time, const units::Velocity& left_velocity, const units::Velocity& right_velocity);
                void updateImu(const units::Time& time, const units::Angle& yaw);
                void updateImu(const units::Time& time, const units::Angle& yaw, const units::Angle& pitch, const units::Angle& roll);
                void updateImu(const units::Time& time, const units::Angle& yaw, const units::Angle& pitch, const units::Angle& roll, const units::AngularVelocity& yaw_velocity, const units::AngularVelocity& pitch_velocity, const units::AngularVelocity& roll_velocity);
                void updateImu(const units::Time& time, const units::Angle& yaw, const units::Angle& pitch, const units::Angle& roll, const units::Acceleration& x, const units::Acceleration& y, const units::Acceleration& z);
                void updateImu(const units::Time& time, const units::Angle& yaw, const units::Angle& pitch, const units::Angle& roll, const units::AngularVelocity& yaw_velocity, const units::AngularVelocity& pitch_velocity, const units::AngularVelocity& roll_velocity, const units::Acceleration& x, const units::Acceleration& y, const units::Acceleration& z);

                units::Distance x();
                units::Distance y();
                units::Distance z();
                units::Angle yaw();
                units::Angle pitch();
                units::Angle roll();
                units::Velocity dx();
                units::Velocity dy();
                units::Velocity dz();
                units::AngularVelocity dYaw();
                units::AngularVelocity dPitch();
                units::AngularVelocity dRoll();
                units::Acceleration ddx();
                units::Acceleration ddy();
                units::Acceleration ddz();

                Pose pose() const;

            private:
                void updatePredictions();

                std::unique_ptr<Ukf> m_ukf;

                std::unique_ptr<Odometry> m_odometry;
                std::unique_ptr<Imu> m_imu;
                units::Time m_last_update;
            };
        }
    }
}

#endif //ROBOT_POSE_HPP
