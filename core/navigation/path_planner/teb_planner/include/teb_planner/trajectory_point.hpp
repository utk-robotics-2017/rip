#ifndef TRAJECTORY_POINT_HPP
#define TRAJECTORY_POINT_HPP

#include <teb_planner/fake_ros_msgs.hpp>
#include <units/units.hpp>
#include <geometry/point.hpp>
#include <json.hpp>

namespace rip
{
    namespace navigation
    {
        namespace tebplanner
        {
            class TrajectoryPoint
            {
            public:
                TrajectoryPoint();

                TrajectoryPoint(const fakeros::TrajectoryPointMsg& msg);

                units::Time t() const;

                void setT(const units::Time& t);

                units::Distance x() const;

                void setX(const units::Distance& x);

                units::Distance y() const;

                void setY(const units::Distance& y);

                geometry::Point position() const;

                void setPosition(const units::Distance& x, const units::Distance& y);

                void setPosition(const geometry::Point& p);

                units::Angle theta() const;

                void setTheta(const units::Angle& theta);

                units::Angle orientation() const;

                void setOrientation(const units::Angle& theta);

                units::Velocity dx() const;

                void setDx(const units::Velocity& dx);

                units::Velocity dy() const;

                void setDy(const units::Velocity& dy);

                units::AngularVelocity omega() const;

                void setOmega(const units::AngularVelocity& omega);

                units::Acceleration ddx() const;

                void setDdx(const units::Acceleration& ddx);

                units::Acceleration ddy() const;

                void setDdy(const units::Acceleration& ddy);

                units::AngularAcceleration alpha() const;

                void setAlpha(const units::AngularAcceleration& alpha);

            private:
                units::Time m_t;
                units::Distance m_x, m_y;
                units::Angle m_theta;
                units::Velocity m_dx, m_dy;
                units::AngularVelocity m_omega;
                units::Acceleration m_ddx, m_ddy;
                units::AngularAcceleration m_alpha;
            };

            void to_json(nlohmann::json& j, const TrajectoryPoint& t);
            void from_json(const nlohmann::json& j, TrajectoryPoint& t);
        }
    }
}

#endif // TRAJECTORY_POINT_HPP
