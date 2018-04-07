#ifndef ROTATION_2D_HPP
#define ROTATION_2D_HPP

#include <units/units.hpp>

#include "path_follower/interpolable.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            class Translation2d;

            class Rotation2d : public Interpolable<Rotation2d>
            {
            public:
                Rotation2d();
                Rotation2d(double x, double y, bool normalize);
                Rotation2d(const Rotation2d& other);
                Rotation2d(const Translation2d& direction, bool normalize);
                Rotation2d(const units::Angle& angle);

                void normalize();

                double cos() const;
                double sin() const;
                double tan() const;

                units::Angle angle() const;

                Rotation2d rotateBy(const Rotation2d& other) const;
                Rotation2d normal() const;
                Rotation2d inverse() const;
                bool parallel(const Rotation2d& other) const;
                Translation2d toTranslation() const;

                Rotation2d interpolate(const Rotation2d& other, double x) override;

                std::string toString() const;
                friend std::ostream& operator<<(std::ostream& os, const Rotation2d& rhs);

            protected:
                double m_cos_angle;
                double m_sin_angle;
            };

            std::ostream& operator<<(std::ostream& os, const Rotation2d& rhs);
        }
    }
}

#endif //ROTATION_2D_HPP
