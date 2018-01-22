#ifndef ACCELERATION_POSE_HPP
#define ACCELERATION_POSE_HPP

#include <units/units.hpp>

namespace rip
{
    namespace navigation
    {
        class AccelerationPose
        {
        public:
            AccelerationPose() = default;

            AccelerationPose(const units::Acceleration& x, const units::Acceleration& y, const units::AngularAcceleration& theta);

            units::Acceleration x() const;

            void setX(const units::Acceleration& x);

            units::Acceleration y() const;

            void setY(const units::Acceleration& y);

            units::AngularAcceleration alpha() const;

            void setAlpha(const units::AngularAcceleration& theta);

            AccelerationPose operator +(const AccelerationPose& rhs) const;

            AccelerationPose& operator +=(const AccelerationPose& rhs);

            AccelerationPose operator -(const AccelerationPose& rhs) const;

            AccelerationPose& operator -=(const AccelerationPose& rhs);

            AccelerationPose operator *(double rhs) const;

            AccelerationPose& operator *=(double rhs);

            bool operator ==(const AccelerationPose& rhs) const;

            bool operator !=(const AccelerationPose& rhs) const;

        private:
            units::Acceleration m_x;
            units::Acceleration m_y;
            units::AngularAcceleration m_alpha;
        };
    }
}

#endif // ACCELERATION_POSE_HPP
