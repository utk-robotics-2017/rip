#ifndef POINT_HPP
#define POINT_HPP

#include <units/units.hpp>
#include <json.hpp>
#include <clipper.hpp>

namespace rip
{
    namespace geometry
    {
        using Distance = units::Distance;
        using Angle = units::Angle;
        /**
         * Basic 2D point class
         */
        class Point
        {
        public:
            Point() = default;

            Point(const ClipperLib::IntPoint& point);

            /**
             * @brief Point
             * @param x
             * @param y
             */
            Point(const Distance& x, const Distance& y);

            /**
             * @brief x
             * @return
             */
            Distance x() const;

            /**
             * @brief x
             * @param x
             */
            void setX(const Distance& x);

            /**
             * @brief y
             * @return
             */
            Distance y() const;

            /**
             * @brief y
             * @param y
             */
            void setY(const Distance& y);

            /**
             * @returns The distance between this point and \p rhs
             */
            Distance distance(const Point& rhs) const;

            /**
             * @returns The distance between this point and the origin
             */
            Distance magnitude() const;

            /**
             * @returns Returns the normalized version of this point
             */
            Point normalize() const;

            double dot(const Point& rhs) const;

            double cross(const Point& rhs) const;

            /**
             * Implicit conversion to {@link ClipperLib::IntPoint}
             */
            operator ClipperLib::IntPoint() const;

            Point operator -() const;
            Point operator +(const Point& rhs) const;
            Point& operator +=(const Point& rhs);
            Point operator -(const Point& rhs) const;
            Point& operator -=(const Point& rhs);
            Point operator *(double rhs) const;
            Point& operator *=(double rhs);
            Point operator /(double rhs) const;
            Point& operator /=(double rhs);
            bool operator ==(const Point& rhs) const;
            bool operator !=(const Point& rhs) const;

            friend void from_json(const nlohmann::json& j, Point& point);
            friend void to_json(nlohmann::json& j, const Point& point);

        private:
            Distance m_x;
            Distance m_y;
        }; // class Point

        Point operator*(double lhs, const Point& rhs);

        Angle atan(const Point& p);

        void from_json(const nlohmann::json& j, Point& point);
        void to_json(nlohmann::json& j, const Point& point);
    }
} // namespace rip

#endif // POINT_HPP
