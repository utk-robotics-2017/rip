#ifndef SHAPE_HPP
#define SHAPE_HPP

#include <units.hpp>

#include "point.hpp"

namespace rip
{
    namespace geometry
    {
        using Area = units::Area;
        /**
         * Abstract base class for shapes
         */
        class Shape
        {
        public:
            Shape() = default;

            /**
             * @brief Determines if the specified point is inside the shape
             */
            virtual bool inside(const Point& point) const = 0;

            /**
             * @returns The perimeter of a shape
             */
            virtual Distance perimeter() const = 0;

            /**
             * @returns The area of a shape
             */
            virtual Area area() const = 0;
        }; // class Shape
    } // namespace geometry
} // namespace rip

#endif // SHAPE_HPP
