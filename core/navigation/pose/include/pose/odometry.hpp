#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

#include <units/units.hpp>

namespace rip
{
    namespace navigation
    {
        namespace pose
        {
            class Odometry
            {
            public:
                Odometry(const units::Distance& wheelbase);

                void update(const units::Time& time, const units::Distance& left, const units::Distance& right);
                void update(const units::Time& time, const units::Velocity& left, const units::Velocity& right);

                units::Time time() const;

                units::Distance x() const;
                units::Distance y() const;
                units::Angle theta() const;

                units::Velocity dx() const;
                units::Velocity dy() const;
                units::AngularVelocity dtheta() const;

                units::Velocity linear() const;
                units::AngularVelocity angular() const;


            private:
                units::Distance m_wheelbase;

                units::Time m_time;

                units::Distance m_x;
                units::Distance m_y;
                units::Angle m_theta;

                units::Velocity m_dx;
                units::Velocity m_dy;
                units::AngularVelocity m_dtheta;

                units::Velocity m_linear;
                units::AngularVelocity m_angular;
            };
        }
    }
}
#endif //ODOMETRY_HPP
