#ifndef CIRCLE_HPP
#define CIRCLE_HPP

#include "shape.hpp"
#include "rectangle.hpp"

namespace rip
{
    namespace geometry
    {
        /**
         * A geometric circle
         */
        class Circle : public Shape
        {
        public:
            Circle() = default;

            /**
             * Constructor
             *
             * @param center The center position of the circle
             * @param radius The radius of the circle
             */
            Circle(const Point& center, const Distance& radius);

            /**
             * Sets the center of the circle
             */
            void center(const Point& center);

            /**
             * @returns The center of the circle
             */
            Point center() const;

            /**
             * Sets the radius of the circle
             */
            void radius(const Distance& radius);

            /**
             * @returns The radius of the circle
             */
            Distance radius() const;

            /**
             * @brief Determines if the specified point is inside the circle
             */
            virtual bool inside(const Point& point) const;

            /**
             * @returns The perimeter of the circle
             */
            virtual Distance perimeter() const;

            /**
             * @returns The area of the circle
             */
            virtual Area area() const;

            Rectangle boundingBox() const;

            friend void from_json(const nlohmann::json& j, Circle& c);
            friend void to_json(nlohmann::json& j, const Circle& c);

        private:
            Point m_center;
            Distance m_radius;
        }; // class Circle

        void from_json(const nlohmann::json& j, Circle& c);
        void to_json(nlohmann::json& j, const Circle& c);
    } // namespace geometry
} // namespace rip

#endif