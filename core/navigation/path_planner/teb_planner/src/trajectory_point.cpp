#include <teb_planner/trajectory_point.hpp>

namespace rip
{
    namespace navigation
    {
        namespace tebplanner
        {

            TrajectoryPoint::TrajectoryPoint()
            {}

            TrajectoryPoint::TrajectoryPoint(const fakeros::TrajectoryPointMsg& msg)
            {
                m_t = msg.time_from_start.sec * units::s + msg.time_from_start.nsec * units::nano * units::s;
                m_x = msg.pose.position.x * units::m;
                m_y = msg.pose.position.y * units::m;
                m_theta = msg.pose.orientation.z * units::rad;
                m_dx = msg.velocity.linear.x * units::m / units::s;
                m_dy = msg.velocity.linear.y  * units::m / units::s;
                m_omega = msg.velocity.angular.z * units::rad / units::s;
                m_ddx = msg.acceleration.linear.x * units::m / units::s / units::s;
                m_ddy = msg.acceleration.linear.y * units::m / units::s / units::s;
                m_alpha = msg.acceleration.angular.z * units::rad / units::s / units::s;
            }

            units::Time TrajectoryPoint::t() const
            {
                return m_t;
            }

            void TrajectoryPoint::setT(const units::Time& t)
            {
                m_t = t;
            }

            units::Distance TrajectoryPoint::x() const
            {
                return m_x;
            }

            void TrajectoryPoint::setX(const units::Distance& x)
            {
                m_x = x;
            }

            units::Distance TrajectoryPoint::y() const
            {
                return m_y;
            }

            void TrajectoryPoint::setY(const units::Distance& y)
            {
                m_y = y;
            }

            geometry::Point TrajectoryPoint::position() const
            {
                return geometry::Point(m_x, m_y);
            }

            void TrajectoryPoint::setPosition(const units::Distance& x, const units::Distance& y)
            {
                m_x = x;
                m_y = y;
            }

            void TrajectoryPoint::setPosition(const geometry::Point& p)
            {
                m_x = p.x();
                m_y = p.y();
            }

            units::Angle TrajectoryPoint::theta() const
            {
                return m_theta;
            }

            void TrajectoryPoint::setTheta(const units::Angle& theta)
            {
                m_theta = theta;
            }

            units::Angle TrajectoryPoint::orientation() const
            {
                return m_theta;
            }

            void TrajectoryPoint::setOrientation(const units::Angle& theta)
            {
                m_theta = theta;
            }

            units::Velocity TrajectoryPoint::dx() const
            {
                return m_dx;
            }

            void TrajectoryPoint::setDx(const units::Velocity& dx)
            {
                m_dx = dx;
            }

            units::Velocity TrajectoryPoint::dy() const
            {
                return m_dy;
            }

            void TrajectoryPoint::setDy(const units::Velocity& dy)
            {
                m_dy = dy;
            }

            units::AngularVelocity TrajectoryPoint::omega() const
            {
                return m_omega;
            }

            void TrajectoryPoint::setOmega(const units::AngularVelocity& omega)
            {
                m_omega = omega;
            }

            units::Acceleration TrajectoryPoint::ddx() const
            {
                return m_ddx;
            }

            void TrajectoryPoint::setDdx(const units::Acceleration& ddx)
            {
                m_ddx = ddx;
            }

            units::Acceleration TrajectoryPoint::ddy() const
            {
                return m_ddy;
            }

            void TrajectoryPoint::setDdy(const units::Acceleration& ddy)
            {
                m_ddy = ddy;
            }

            units::AngularAcceleration TrajectoryPoint::alpha() const
            {
                return m_alpha;
            }

            void TrajectoryPoint::setAlpha(const units::AngularAcceleration& alpha)
            {
                m_alpha = alpha;
            }

            void to_json(nlohmann::json& j, const TrajectoryPoint& t)
            {
                j["t"] = t.t();
                j["x"] = t.x();
                j["y"] = t.y();
                j["theta"] = t.theta();
                j["dx"] = t.dx();
                j["dy"] = t.dy();
                j["omega"] = t.omega();
                j["ddx"] = t.ddx();
                j["ddy"] = t.ddy();
                j["alpha"] = t.alpha();
            }

            void from_json(const nlohmann::json& j, TrajectoryPoint& t)
            {
                t.setT(j["t"]);
                t.setX(j["x"]);
                t.setY(j["y"]);
                t.setTheta(j["theta"]);
                t.setDx(j["dx"]);
                t.setDdy(j["dy"]);
                t.setOmega(j["omega"]);
                t.setDdx(j["ddx"]);
                t.setDdy(j["ddy"]);
                t.setAlpha(j["alpha"]);
            }
        }
    }
}
