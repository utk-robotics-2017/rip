#ifndef POLYGON_LIST_HPP
#define POLYGON_LIST_HPP

#include "shape.hpp"
#include "polygon.hpp"
#include "rectangle.hpp"

#include <clipper.hpp>
#include <json.hpp>

namespace rip
{
    namespace geometry
    {
        class Polygon;

        /**
         * A list of {@link Polygon} object where the orientation determines if
         * the polygon is an exterior or a hole.
         */
        class PolygonList : public Shape
        {
        public:
            PolygonList() = default;

            PolygonList(const std::vector< Polygon >& rhs);

            std::vector< PolygonList > splitIntoIslands() const;

        private:
            void splitIntoIslands_processPolyTreeNodes(ClipperLib::PolyNode* node, std::vector< PolygonList >& rv) const;

        public:
            virtual bool inside(const Point& point) const override;

            virtual Area area() const override;

            PolygonList offset(const Distance& rhs) const;

            Rectangle boundingBox() const;

            std::vector<Polygon>::iterator begin();

            std::vector<Polygon>::iterator end();

            Polygon operator [](int index) const;

            size_t size() const;

            PolygonList operator +(const Polygon& rhs) const;

            PolygonList operator +(const PolygonList& rhs) const;

            PolygonList& operator +=(const Polygon& rhs);

            PolygonList& operator +=(const PolygonList& rhs);

            PolygonList operator -(const Polygon& rhs) const;

            PolygonList operator -(const PolygonList& rhs) const;

            PolygonList& operator -=(const Polygon& rhs);

            PolygonList& operator -=(const PolygonList& rhs);

            PolygonList operator &(const Polygon& rhs) const;

            PolygonList operator &(const PolygonList& rhs) const;

            PolygonList& operator &=(const Polygon& rhs);

            PolygonList& operator &=(const PolygonList& rhs);

            PolygonList operator |(const Polygon& rhs) const;

            PolygonList operator |(const PolygonList& rhs) const;

            PolygonList& operator |=(const Polygon& rhs);

            PolygonList& operator |=(const PolygonList& rhs);

            PolygonList operator ^(const Polygon& rhs) const;

            PolygonList operator ^(const PolygonList& rhs) const;

            PolygonList& operator ^=(const Polygon& rhs);

            PolygonList& operator ^=(const PolygonList& rhs);

            friend void from_json(const nlohmann::json& j, PolygonList& polygons);

            friend void to_json(nlohmann::json& j, const PolygonList& polygons);

        private:

            // Isn't used
            virtual Distance perimeter() const override {return 0.0;}

            PolygonList _add(const Polygon& rhs) const;

            PolygonList _add(const PolygonList& rhs) const;

            PolygonList& _add_to_this(const Polygon& rhs);

            PolygonList& _add_to_this(const PolygonList& rhs);

            PolygonList _subtract(const Polygon& rhs) const;

            PolygonList _subtract(const PolygonList& rhs) const;

            PolygonList& _subtract_from_this(const Polygon& rhs);

            PolygonList& _subtract_from_this(const PolygonList& rhs);

            PolygonList _intersection(const Polygon& rhs) const;

            PolygonList _intersection(const PolygonList& rhs) const;

            PolygonList& _intersection_with_this(const Polygon& rhs);

            PolygonList& _intersection_with_this(const PolygonList& rhs);

            PolygonList _xor(const Polygon& rhs) const;

            PolygonList _xor(const PolygonList& rhs) const;

            PolygonList& _xor_with_this(const Polygon& rhs);

            PolygonList& _xor_with_this(const PolygonList& rhs);

            operator ClipperLib::Paths() const;

            PolygonList(const ClipperLib::Paths& paths);

            void load(const ClipperLib::Paths& paths);

            std::vector< Polygon > m_polygons;

            friend class Polygon;
        }; // class PolygonList

        void from_json(const nlohmann::json& j, PolygonList& polygons);
        void to_json(nlohmann::json& j, const PolygonList& polygons);

    } // namespace geometry
} // namespace rip


#endif // POLYGON_LIST_HPP