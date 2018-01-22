#ifndef VELOCITY_POINT_HPP
#define VELOCITY_POINT_HPP

#include <units/units.hpp>
#include <json.hpp>

namespace rip
{

    namespace navigation
    {
        using Velocity = units::Velocity;
        using Angle = units::Angle;
        using AngularVelocity = units::AngularVelocity;
        /**
         * Basic 2D point class
         */
        class VelocityPose
        {
        public:
            VelocityPose() = default;

            /**
             * @brief Point
             * @param x
             * @param y
             */
            VelocityPose(const Velocity& x, const Velocity& y, const AngularVelocity& omega);

            /**
             * @brief x
             * @return
             */
            Velocity x() const;

            /**
             * @brief x
             * @param x
             */
            void setX(const Velocity& x);

            /**
             * @brief y
             * @return
             */
            Velocity y() const;

            /**
             * @brief y
             * @param y
             */
            void setY(const Velocity& y);

            AngularVelocity omega() const;

            void setOmega(const AngularVelocity& omega);

            VelocityPose operator -() const;
            VelocityPose operator +(const VelocityPose& rhs) const;
            VelocityPose& operator +=(const VelocityPose& rhs);
            VelocityPose operator -(const VelocityPose& rhs) const;
            VelocityPose& operator -=(const VelocityPose& rhs);
            VelocityPose operator *(double rhs) const;
            VelocityPose& operator *=(double rhs);
            VelocityPose operator /(double rhs) const;
            VelocityPose& operator /=(double rhs);

            bool operator ==(const VelocityPose& rhs) const;

            bool operator !=(const VelocityPose& rhs) const;

            friend void from_json(const nlohmann::json& j, VelocityPose& point);
            friend void to_json(nlohmann::json& j, const VelocityPose& point);

        private:
            Velocity m_x;
            Velocity m_y;
            AngularVelocity m_omega;
        }; // class Point

        VelocityPose operator*(double lhs, const VelocityPose& rhs);

        Angle atan(const VelocityPose& p);

        void from_json(const nlohmann::json& j, VelocityPose& point);
        void to_json(nlohmann::json& j, const VelocityPose& point);
    }
}

#endif // VELOCITY_POINT_HPP
