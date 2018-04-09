#ifndef TRANSLATION_2D_HPP
#define TRANSLATION_2D_HPP

#include <units/units.hpp>
#include <geometry/point.hpp>

#include "path_follower/interpolable.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            class Rotation2d;

            /**
             * A translation in a 2d coordinate frame. Translations are simply shifts in an (x, y) plane.
             */
            class Translation2d : public Interpolable<Translation2d>
            {
            public:
                static Translation2d identity();

                Translation2d();
                Translation2d(const units::Distance& x, const units::Distance& y);
                Translation2d(const Translation2d& other);
                Translation2d(const Translation2d& start, const Translation2d& end);

                /**
                 * The "norm" of a transform is the Euclidean distance in x and y.
                 *
                 * @return sqrt(x^2 + y^2)
                 */
                units::Distance norm() const;
                double norm2() const;

                units::Distance x() const;
                units::Distance y() const;

                void setX(const units::Distance& x);
                void setY(const units::Distance& y);

                /**
                 * We can compose Translation2d's by adding together the x and y shifts.
                 *
                 * @param other
                 *            The other translation to add.
                 * @return The combined effect of translating by this object and the other.
                 */
                Translation2d translateBy(const Translation2d& other) const;

                /**
                 * We can also rotate Translation2d's. See: https://en.wikipedia.org/wiki/Rotation_matrix
                 *
                 * @param rotation
                 *            The rotation to apply.
                 * @return This translation rotated by rotation.
                 */
                Translation2d rotateBy(const Rotation2d& rotation) const;
                Translation2d rotateBy(const units::Angle& angle) const;

                Rotation2d direction() const;

                /**
                 * The inverse simply means a Translation2d that "undoes" this object.
                 *
                 * @return Translation by -x and -y.
                 */
                Translation2d inverse() const;

                Translation2d perpendicular() const;

                Translation2d interpolate(const Translation2d& other, double x) override;
                Translation2d extrapolate(const Translation2d& other, double x);
                Translation2d scale(double s) const;

                std::string toString() const;
                friend std::ostream& operator<<(std::ostream&, const Translation2d&);

                double dot(const Translation2d& other);
                static double dot(const Translation2d& lhs, const Translation2d& rhs);

                static units::Angle getAngle(const Translation2d& lhs, const Translation2d& rhs);

                double cross(const Translation2d& rhs);
                static double cross(const Translation2d& lhs, const Translation2d& rhs);

                operator geometry::Point() const;

                Translation2d operator-(const Translation2d& rhs) const;
                Translation2d& operator-=(const Translation2d& rhs);
                Translation2d operator+(const Translation2d& rhs) const;
                Translation2d operator+=(const Translation2d& rhs);

            protected:
                units::Distance m_x;
                units::Distance m_y;
            };

            std::ostream& operator<<(std::ostream&, const Translation2d&);
        }
    }
}

#endif //TRANSLATION_2D_HPP
