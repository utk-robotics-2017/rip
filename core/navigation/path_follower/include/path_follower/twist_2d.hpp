#ifndef TWIST_2D_HPP
#define TWIST_2D_HPP

#include <units/units.hpp>

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            /**
             * A movement along an arc at constant curvature and velocity. We can use ideas from "differential calculus" to create
             * new RigidTransform2d's from a Twist2d and visa versa.
             *
             * A Twist can be used to represent a difference between two poses, a velocity, an acceleration, etc.
             */
            class Twist2d
            {
            public:
                Twist2d();
                Twist2d(const units::Distance& dx, const units::Distance& dy, const units::Angle& dtheta);

                units::Distance dx() const;
                units::Distance dy() const;
                units::Angle dtheta() const;

                void setDx(const units::Distance& dx);
                void setDy(const units::Distance& dy);
                void setDtheta(const units::Angle& dtheta);

                Twist2d scaled(double scale) const;
            private:
                units::Distance m_dx;
                units::Distance m_dy;
                units::Angle m_dtheta;
            };
        }
    }
}

#endif //TWIST_2D_HPP
