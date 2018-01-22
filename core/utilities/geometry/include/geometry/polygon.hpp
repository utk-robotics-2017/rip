#ifndef POLYGON_HPP
#define POLYGON_HPP

#include "shape.hpp"
#include "polygon_list.hpp"
#include "rectangle.hpp"

#include <clipper.hpp>
#include <json.hpp>

namespace rip
{
    namespace geometry
    {
        class PolygonList;

        /**
         * A single polygon
         */
        class Polygon : public Shape
        {
        public:
            /**
             *  Default Constructor
             */
            Polygon() = default;

            /**
             * Constructor
             * @param points The list of points making up the polygon
             */
            Polygon(const std::vector< Point >& points);

            void add(const Point& rhs);

            /**
             * @returns Whether the point is inside the polygon (on the edge counts as inside)
             */
            virtual bool inside(const Point& point) const override;

            /**
             * @returns The perimeter of the polygon
             */
            virtual Distance perimeter() const override;

            /**
             * @returns The area of the polygon
             */
            virtual Area area() const override;

            Point centroid() const;

            enum class Orientation
            {
                kClockwise,
                kCounterClockwise
            };

            /**
             * @returns Whether the polygon is clockwise or counter-clockwise
             */
            Orientation orientation() const;

            /**
             * @returns Whether this is an exterior polygon (as opposed to a hole)
             */
            bool exterior() const;

            /**
             * @returns Whether this is a hole
             */
            bool hole() const;

            Distance minX() const;

            Distance maxX() const;

            Distance minY() const;

            Distance maxY() const;

            Rectangle boundingBox() const;

            /**
             * @returns A polygon that has been offset by the specified amount
             */
            PolygonList offset(const Distance& rhs) const;

            Point& front()
            {
                return m_points.front();
            }

            const Point& front() const
            {
                return m_points.front();
            }

            Point& back()
            {
                return m_points.back();
            }

            const Point& back() const
            {
                return m_points.back();
            }

            std::vector<Point>::const_iterator begin() const;

            std::vector<Point>::const_iterator end() const;

            std::vector<Point>::iterator begin();

            std::vector<Point>::iterator end();

            Point operator [](int index) const;

            size_t size() const;

            /**
             * Combines this polygon with another
             *
             * @returns The {@link PolygonList} that results from the union
             */
            PolygonList operator +(const Polygon& rhs) const;

            /**
             * Combines this polygon with a list of polygons
             *
             * @returns The {@link PolygonList} that results from the union
             */
            PolygonList operator +(const PolygonList& rhs) const;

            /**
             * Adds a new point to this polygon at the end
             */
            PolygonList& operator +=(const Point& rhs);

            /**
             * Subtracts another polygon from this polygon
             *
             * @returns The {@link PolygonList} that results from the difference
             */
            PolygonList operator -(const Polygon& rhs) const;

            /**
             * Subtracts a list of polygons from this polygon
             *
             * @returns The {@link PolygonList} that results from the difference
             */
            PolygonList operator -(const PolygonList& rhs) const;

            /**
             * Creates the intersect of another polygon and this polygon
             *
             * @returns The {@link PolygonList} that results from the intersection
             */
            PolygonList operator &(const Polygon& rhs) const;

            /**
             * Creates the intersect of a list of polygons and this polygon
             *
             * @returns The {@link PolygonList} that results from the intersection
             */
            PolygonList operator &(const PolygonList& rhs) const;

            /**
             * Combines this polygon with another
             *
             * @returns The {@link PolygonList} that results from the union
             */
            PolygonList operator |(const Polygon& rhs) const;

            /**
             * Combines this polygon with a list of polygons
             *
             * @returns The {@link PolygonList} that results from the union
             */
            PolygonList operator |(const PolygonList& rhs) const;

            /**
             * Creates the xor of another polygon and this polygon
             *
             * @returns The {@link PolygonList} that results from the xor
             */
            PolygonList operator ^(const Polygon& rhs) const;

            /**
             * Creates the intersect of a list of polygons and this polygon
             *
             * @returns The {@link PolygonList} that results from the xor
             */
            PolygonList operator ^(const PolygonList& rhs) const;

            friend void from_json(const nlohmann::json& j, Polygon& polygon);

            friend void to_json(nlohmann::json& j, const Polygon& polygon);

        private:

            PolygonList _add(const Polygon& rhs) const;

            PolygonList _add(const PolygonList& rhs) const;

            PolygonList _subtract(const Polygon& rhs) const;

            PolygonList _subtract(const PolygonList& rhs) const;

            PolygonList _intersection(const Polygon& rhs) const;

            PolygonList _intersection(const PolygonList& rhs) const;

            PolygonList _xor(const Polygon& rhs) const;

            PolygonList _xor(const PolygonList& rhs) const;

            operator ClipperLib::Path() const;

            Polygon(const ClipperLib::Path& path);

            std::vector< Point > m_points;

            friend class PolygonList;
        }; // class Polygon

        void from_json(const nlohmann::json& j, Polygon& polygon);
        void to_json(nlohmann::json& j, const Polygon& polygon);
    } // namespace geometry
} // namespace rip

#endif // POLYGON_HPP
