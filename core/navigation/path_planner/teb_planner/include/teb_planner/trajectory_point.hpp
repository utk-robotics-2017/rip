#ifndef TRAJECTORY_POINT_HPP
#define TRAJECTORY_POINT_HPP

#include <units/units.hpp>

#include "pose.hpp"
#include "velocity_pose.hpp"
#include "acceleration_pose.hpp"

namespace rip
{
    namespace navigation
    {
        class TrajectoryPoint
        {
        public:
            TrajectoryPoint() = default;

            TrajectoryPoint(const Pose& pose, const VelocityPose& velocity, const AccelerationPose& acceleration);

            units::Time t() const;

            void setT(const units::Time& t);

            void setTime(const units::Time& t);

            Pose pose() const;

            void setPose(const Pose& pose);

            void setPose(const units::Distance& x, const units::Distance& y, const units::Angle& theta);

            geometry::Point position() const;

            units::Distance x() const;

            void setX(const units::Distance& x);

            units::Distance y() const;

            void setY(const units::Distance& y);

            units::Angle theta() const;

            void setTheta(const units::Angle& theta);

            VelocityPose velocity() const;

            void setVelocity(const VelocityPose& velocity);

            void setVelocity(const units::Velocity& dx, const units::Velocity& dy, const units::AngularVelocity& omega);

            units::Velocity dx() const;

            void setDx(const units::Velocity& dx);

            units::Velocity dy() const;

            void setDy(const units::Velocity& dy);

            units::AngularVelocity omega() const
            {
                return m_velocity.omega();
            }

            void setOmega(const units::AngularVelocity& omega);

            units::AngularVelocity dTheta() const;

            AccelerationPose acceleration() const;

            void setAcceleration(const AccelerationPose& acceleration);

            void setAcceleration(const units::Acceleration& ddx, const units::Acceleration& ddy, const units::AngularAcceleration& alpha);

            units::Acceleration ddx() const;

            void setDdx(const units::Acceleration& ddx);

            units::Acceleration ddy() const;

            void setDdy(const units::Acceleration& ddy);

            units::AngularAcceleration alpha() const;

            void setAlpha(const units::AngularAcceleration& alpha);

            units::AngularAcceleration dOmega() const;

            units::AngularAcceleration dDTheta() const;

        private:
            units::Time m_t;
            Pose m_pose;
            VelocityPose m_velocity;
            AccelerationPose m_acceleration;
        };
    }
}

#endif // TRAJECTORY_POINT_HPP
