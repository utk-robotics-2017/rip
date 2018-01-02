#ifndef RECTANGLE_HPP
#define RECTANGLE_HPP

#include "shape.hpp"

namespace rip
{
    namespace geometry
    {
        /**
         * A rectangle
         */
        class Rectangle : public Shape
        {
        public:
            Rectangle(const Point& top_left, const Point& bottom_right);

            Rectangle(const Distance& min_x, const Distance& max_x, const Distance& min_y, const Distance& max_y);

            virtual bool inside(const Point& point) const;

            virtual Distance perimeter() const;

            virtual Area area() const;

            Distance width() const;

            Distance height() const;

            Point center() const;

            Point topLeft() const;

            Point topRight() const;

            Point bottomLeft() const;

            Point bottomRight() const;

            Distance minX() const;

            Distance maxX() const;

            Distance minY() const;

            Distance maxY() const;

        private:
            Point m_top_left;
            Point m_bottom_right;
        }; //class Rectangle
    } // namespace geometry
} // namespace rip

#endif // RECTANGLE_HPP