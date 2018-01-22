#include <teb_planner/trajectory_point.hpp>

namespace rip
{
    namespace navigation
    {

        TrajectoryPoint::TrajectoryPoint(const Pose& pose, const VelocityPose& velocity, const AccelerationPose& acceleration)
            : m_pose(pose)
            , m_velocity(velocity)
            , m_acceleration(acceleration)
        {}

        units::Time TrajectoryPoint::t() const
        {
            return m_t;
        }

        void TrajectoryPoint::setT(const units::Time& t)
        {
            m_t = t;
        }

        void TrajectoryPoint::setTime(const units::Time& t)
        {
            setT(t);
        }

        Pose TrajectoryPoint::pose() const
        {
            return m_pose;
        }

        void TrajectoryPoint::setPose(const Pose& pose)
        {
            m_pose = pose;
        }

        void TrajectoryPoint::setPose(const units::Distance& x, const units::Distance& y, const units::Angle& theta)
        {
            m_pose = Pose(x, y, theta);
        }

        geometry::Point TrajectoryPoint::position() const
        {
            return m_pose.position();
        }

        units::Distance TrajectoryPoint::x() const
        {
            return m_pose.x();
        }

        void TrajectoryPoint::setX(const units::Distance& x)
        {
            m_pose.setX(x);
        }

        units::Distance TrajectoryPoint::y() const
        {
            return m_pose.y();
        }

        void TrajectoryPoint::setY(const units::Distance& y)
        {
            m_pose.setY(y);
        }

        units::Angle TrajectoryPoint::theta() const
        {
            return m_pose.theta();
        }

        void TrajectoryPoint::setTheta(const units::Angle& theta)
        {
            m_pose.setTheta(theta);
        }

        VelocityPose TrajectoryPoint::velocity() const
        {
            return m_velocity;
        }

        void TrajectoryPoint::setVelocity(const VelocityPose& velocity)
        {
            m_velocity = velocity;
        }

        void TrajectoryPoint::setVelocity(const units::Velocity& dx, const units::Velocity& dy, const units::AngularVelocity& omega)
        {
            m_velocity = VelocityPose(dx, dy, omega);
        }

        units::Velocity TrajectoryPoint::dx() const
        {
            return m_velocity.x();
        }

        void TrajectoryPoint::setDx(const units::Velocity& dx)
        {
            m_velocity.setX(dx);
        }

        units::Velocity TrajectoryPoint::dy() const
        {
            return m_velocity.y();
        }

        void TrajectoryPoint::setDy(const units::Velocity& dy)
        {
            m_velocity.setY(dy);
        }

        void TrajectoryPoint::setOmega(const units::AngularVelocity& omega)
        {
            m_velocity.setOmega(omega);
        }

        units::AngularVelocity TrajectoryPoint::dTheta() const
        {
            return m_velocity.omega();
        }

        AccelerationPose TrajectoryPoint::acceleration() const
        {
            return m_acceleration;
        }

        void TrajectoryPoint::setAcceleration(const AccelerationPose& acceleration)
        {
            m_acceleration = acceleration;
        }

        void TrajectoryPoint::setAcceleration(const units::Acceleration& ddx, const units::Acceleration& ddy, const units::AngularAcceleration& alpha)
        {
            m_acceleration = AccelerationPose(ddx, ddy, alpha);
        }

        units::Acceleration TrajectoryPoint::ddx() const
        {
            return m_acceleration.x();
        }

        void TrajectoryPoint::setDdx(const units::Acceleration& ddx)
        {
            m_acceleration.setX(ddx);
        }

        units::Acceleration TrajectoryPoint::ddy() const
        {
            return m_acceleration.y();
        }

        void TrajectoryPoint::setDdy(const units::Acceleration& ddy)
        {
            m_acceleration.setY(ddy);
        }

        units::AngularAcceleration TrajectoryPoint::alpha() const
        {
            return m_acceleration.alpha();
        }

        void TrajectoryPoint::setAlpha(const units::AngularAcceleration& alpha)
        {
            m_acceleration.setAlpha(alpha);
        }

        units::AngularAcceleration TrajectoryPoint::dOmega() const
        {
            return m_acceleration.alpha();
        }

        units::AngularAcceleration TrajectoryPoint::dDTheta() const
        {
            return m_acceleration.alpha();
        }

    }
}
