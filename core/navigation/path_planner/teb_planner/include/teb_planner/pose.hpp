#ifndef POSE_HPP
#define POSE_HPP

#include <geometry/point.hpp>

namespace rip
{
    namespace navigation
    {
        /**
         * Implements a pose in the SE2 domain: \f$ \mathbb{R}^2 \ times S^1\f$
         *
         * The pose consists of the x and y position and the orientation given as
         * angle theta [-pi, pi]
         */
        class Pose
        {
        public:
            Pose() = default;

            Pose(const units::Distance& x, const units::Distance& y, const units::Angle& theta);

            Pose(const geometry::Point& xy, const units::Angle& theta);

            geometry::Point position() const;

            units::Distance x() const;

            void setX(const units::Distance& x);

            units::Distance y() const;

            void setY(const units::Distance& y);

            units::Angle orientation() const;

            geometry::Point orientationUnitVector() const;

            units::Angle theta() const;

            void setTheta(const units::Angle& theta);

            Pose operator +(const Pose& rhs) const;

            Pose& operator +=(const Pose& rhs);

            Pose operator -(const Pose& rhs) const;

            Pose& operator -=(const Pose& rhs);

            Pose operator *(double rhs) const;

            Pose& operator *=(double rhs);

            Pose operator /(double rhs) const;

            Pose operator /=(double rhs);

            bool operator ==(const Pose& rhs) const;

            bool operator !=(const Pose& rhs) const;

        private:
            units::Distance m_x;
            units::Distance m_y;
            units::Angle m_theta;
        };
    }
}

#endif // POSE_HPP
