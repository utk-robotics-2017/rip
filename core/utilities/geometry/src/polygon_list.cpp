#include "geometry/polygon_list.hpp"

namespace rip
{
    namespace geometry
    {
        PolygonList::PolygonList(const std::vector< Polygon >& polygons)
            : m_polygons(polygons)
        {}

        std::vector< PolygonList > PolygonList::splitIntoIslands() const
        {
            std::vector< PolygonList > rv;
            ClipperLib::Clipper clipper(0);
            ClipperLib::PolyTree poly_tree;
            clipper.AddPaths(*this, ClipperLib::ptSubject, true);
            clipper.Execute(ClipperLib::ctUnion, poly_tree);

            splitIntoIslands_processPolyTreeNodes(&poly_tree, rv);
            return rv;
        }

        void PolygonList::splitIntoIslands_processPolyTreeNodes(ClipperLib::PolyNode* node, std::vector< PolygonList >& rv) const
        {
            for (int n = 0; n < node->ChildCount(); n++)
            {
                ClipperLib::PolyNode* child = node->Childs[n];
                PolygonList island;
                island += child->Contour;
                for (int i = 0; i < child->ChildCount(); i++)
                {
                    island += child->Childs[i]->Contour;
                    splitIntoIslands_processPolyTreeNodes(child->Childs[i], rv);
                }
                rv.push_back(island);
            }
        }


        bool PolygonList::inside(const Point& rhs) const
        {
            int poly_count_inside = 0;
            for (const ClipperLib::Path& polygon : m_polygons)
            {
                const int is_inside_this_poly = ClipperLib::PointInPolygon(rhs, polygon);

                // On the edge counts as inside
                if (is_inside_this_poly == -1)
                {
                    return true;
                }
                poly_count_inside += is_inside_this_poly;
            }
            return (poly_count_inside % 2) == 1;
        }


        Area PolygonList::area() const
        {
            Area rv = 0;
            // Exterior polygons will have a positive area and holes will have a negative area
            for (const Polygon& polygon : m_polygons)
            {
                rv += polygon.area();
            }
            return rv;
        }

        PolygonList PolygonList::offset(const Distance& rhs) const
        {
            ClipperLib::Paths paths;
            ClipperLib::ClipperOffset clipper;
            clipper.AddPaths(*this, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
            clipper.Execute(paths, rhs());
            return PolygonList(paths);
        }

        Rectangle PolygonList::boundingBox() const
        {
            Distance min_x = std::numeric_limits<double>::max();
            Distance max_x = std::numeric_limits<double>::min();
            Distance min_y = std::numeric_limits<double>::max();
            Distance max_y = std::numeric_limits<double>::min();

            for (const Polygon& polygon : m_polygons)
            {
                Rectangle polygon_bounding_box = polygon.boundingBox();
                if (polygon_bounding_box.minX() < min_x)
                {
                    min_x = polygon_bounding_box.minX();
                }

                if (polygon_bounding_box.maxX() > max_x)
                {
                    max_x = polygon_bounding_box.maxX();
                }

                if (polygon_bounding_box.minY() < min_y)
                {
                    min_y = polygon_bounding_box.minY();
                }

                if (polygon_bounding_box.maxY() > max_y)
                {
                    max_y = polygon_bounding_box.maxY();
                }
            }
            return Rectangle(min_x, max_x, min_y, max_y);
        }

        std::vector<Polygon>::iterator PolygonList::begin()
        {
            return m_polygons.begin();
        }

        std::vector<Polygon>::iterator PolygonList::end()
        {
            return m_polygons.end();
        }

        Polygon PolygonList::operator [](int index) const
        {
            return m_polygons[index];
        }

        size_t PolygonList::size() const
        {
            return m_polygons.size();
        }

        PolygonList PolygonList::operator +(const Polygon& rhs) const
        {
            return _add(rhs);
        }

        PolygonList PolygonList::operator +(const PolygonList& rhs) const
        {
            return _add(rhs);
        }

        PolygonList& PolygonList::operator +=(const Polygon& rhs)
        {
            return _add_to_this(rhs);
        }

        PolygonList& PolygonList::operator +=(const PolygonList& rhs)
        {
            return _add_to_this(rhs);
        }

        PolygonList PolygonList::operator -(const Polygon& rhs) const
        {
            return _subtract(rhs);
        }

        PolygonList PolygonList::operator -(const PolygonList& rhs) const
        {
            return _subtract(rhs);
        }

        PolygonList& PolygonList::operator -=(const Polygon& rhs)
        {
            return _subtract_from_this(rhs);
        }

        PolygonList& PolygonList::operator -=(const PolygonList& rhs)
        {
            return _subtract_from_this(rhs);
        }

        PolygonList PolygonList::operator &(const Polygon& rhs) const
        {
            return _intersection(rhs);
        }

        PolygonList PolygonList::operator &(const PolygonList& rhs) const
        {
            return _intersection(rhs);
        }

        PolygonList& PolygonList::operator &=(const Polygon& rhs)
        {
            return _intersection_with_this(rhs);
        }

        PolygonList& PolygonList::operator &=(const PolygonList& rhs)
        {
            return _intersection_with_this(rhs);
        }

        PolygonList PolygonList::operator |(const Polygon& rhs) const
        {
            return _add(rhs);
        }

        PolygonList PolygonList::operator |(const PolygonList& rhs) const
        {
            return _add(rhs);
        }

        PolygonList& PolygonList::operator |=(const Polygon& rhs)
        {
            return _add_to_this(rhs);
        }

        PolygonList& PolygonList::operator |=(const PolygonList& rhs)
        {
            return _add_to_this(rhs);
        }

        PolygonList PolygonList::operator ^(const Polygon& rhs) const
        {
            return _xor(rhs);
        }

        PolygonList PolygonList::operator ^(const PolygonList& rhs) const
        {
            return _xor(rhs);
        }

        PolygonList& PolygonList::operator ^=(const Polygon& rhs)
        {
            return _xor_with_this(rhs);
        }

        PolygonList& PolygonList::operator ^=(const PolygonList& rhs)
        {
            return _xor_with_this(rhs);
        }

        PolygonList PolygonList::_add(const Polygon& rhs) const
        {
            ClipperLib::Paths paths;
            ClipperLib::Clipper clipper;
            clipper.AddPaths(*this, ClipperLib::ptSubject, true);
            clipper.AddPath(rhs, ClipperLib::ptSubject, true);
            clipper.Execute(ClipperLib::ctUnion, paths, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
            return PolygonList(paths);
        }

        PolygonList PolygonList::_add(const PolygonList& rhs) const
        {
            ClipperLib::Paths paths;
            ClipperLib::Clipper clipper;
            clipper.AddPaths(*this, ClipperLib::ptSubject, true);
            clipper.AddPaths(rhs, ClipperLib::ptSubject, true);
            clipper.Execute(ClipperLib::ctUnion, paths, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
            return PolygonList(paths);
        }

        PolygonList& PolygonList::_add_to_this(const Polygon& rhs)
        {
            ClipperLib::Paths paths;
            ClipperLib::Clipper clipper;
            clipper.AddPaths(*this, ClipperLib::ptSubject, true);
            clipper.AddPath(rhs, ClipperLib::ptSubject, true);
            clipper.Execute(ClipperLib::ctUnion, paths, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
            load(paths);
            return *this;
        }

        PolygonList& PolygonList::_add_to_this(const PolygonList& rhs)
        {
            ClipperLib::Paths paths;
            ClipperLib::Clipper clipper;
            clipper.AddPaths(*this, ClipperLib::ptSubject, true);
            clipper.AddPaths(rhs, ClipperLib::ptSubject, true);
            clipper.Execute(ClipperLib::ctUnion, paths, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
            load(paths);
            return *this;
        }

        PolygonList PolygonList::_subtract(const Polygon& rhs) const
        {
            ClipperLib::Paths paths;
            ClipperLib::Clipper clipper;
            clipper.AddPaths(*this, ClipperLib::ptSubject, true);
            clipper.AddPath(rhs, ClipperLib::ptClip, true);
            clipper.Execute(ClipperLib::ctDifference, paths, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
            return PolygonList(paths);
        }

        PolygonList PolygonList::_subtract(const PolygonList& rhs) const
        {
            ClipperLib::Paths paths;
            ClipperLib::Clipper clipper;
            clipper.AddPaths(*this, ClipperLib::ptSubject, true);
            clipper.AddPaths(rhs, ClipperLib::ptClip, true);
            clipper.Execute(ClipperLib::ctDifference, paths, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
            return PolygonList(paths);
        }

        PolygonList& PolygonList::_subtract_from_this(const Polygon& rhs)
        {
            ClipperLib::Paths paths;
            ClipperLib::Clipper clipper;
            clipper.AddPaths(*this, ClipperLib::ptSubject, true);
            clipper.AddPath(rhs, ClipperLib::ptClip, true);
            clipper.Execute(ClipperLib::ctDifference, paths, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
            load(paths);
            return *this;
        }

        PolygonList& PolygonList::_subtract_from_this(const PolygonList& rhs)
        {
            ClipperLib::Paths paths;
            ClipperLib::Clipper clipper;
            clipper.AddPaths(*this, ClipperLib::ptSubject, true);
            clipper.AddPaths(rhs, ClipperLib::ptClip, true);
            clipper.Execute(ClipperLib::ctDifference, paths, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
            load(paths);
            return *this;
        }

        PolygonList PolygonList::_intersection(const Polygon& rhs) const
        {
            ClipperLib::Clipper clipper;
            ClipperLib::Paths paths;
            clipper.AddPaths(*this, ClipperLib::ptSubject, true);
            clipper.AddPath(rhs, ClipperLib::ptClip, true);
            clipper.Execute(ClipperLib::ctIntersection, paths, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
            return PolygonList(paths);
        }

        PolygonList PolygonList::_intersection(const PolygonList& rhs) const
        {
            ClipperLib::Clipper clipper;
            ClipperLib::Paths paths;
            clipper.AddPaths(*this, ClipperLib::ptSubject, true);
            clipper.AddPaths(rhs, ClipperLib::ptClip, true);
            clipper.Execute(ClipperLib::ctIntersection, paths, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
            return PolygonList(paths);
        }

        PolygonList& PolygonList::_intersection_with_this(const Polygon& rhs)
        {
            ClipperLib::Clipper clipper;
            ClipperLib::Paths paths;
            clipper.AddPaths(*this, ClipperLib::ptSubject, true);
            clipper.AddPath(rhs, ClipperLib::ptClip, true);
            clipper.Execute(ClipperLib::ctIntersection, paths, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
            load(paths);
            return *this;
        }

        PolygonList& PolygonList::_intersection_with_this(const PolygonList& rhs)
        {
            ClipperLib::Clipper clipper;
            ClipperLib::Paths paths;
            clipper.AddPaths(*this, ClipperLib::ptSubject, true);
            clipper.AddPaths(rhs, ClipperLib::ptClip, true);
            clipper.Execute(ClipperLib::ctIntersection, paths, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
            load(paths);
            return *this;
        }

        PolygonList PolygonList::_xor(const Polygon& rhs) const
        {
            ClipperLib::Clipper clipper;
            ClipperLib::Paths paths;
            clipper.AddPaths(*this, ClipperLib::ptSubject, true);
            clipper.AddPath(rhs, ClipperLib::ptClip, true);
            clipper.Execute(ClipperLib::ctXor, paths, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
            return PolygonList(paths);
        }

        PolygonList PolygonList::_xor(const PolygonList& rhs) const
        {
            ClipperLib::Clipper clipper;
            ClipperLib::Paths paths;
            clipper.AddPaths(*this, ClipperLib::ptSubject, true);
            clipper.AddPaths(rhs, ClipperLib::ptClip, true);
            clipper.Execute(ClipperLib::ctXor, paths, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
            return PolygonList(paths);
        }

        PolygonList& PolygonList::_xor_with_this(const Polygon& rhs)
        {
            ClipperLib::Clipper clipper;
            ClipperLib::Paths paths;
            clipper.AddPaths(*this, ClipperLib::ptSubject, true);
            clipper.AddPath(rhs, ClipperLib::ptClip, true);
            clipper.Execute(ClipperLib::ctXor, paths, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
            load(paths);
            return *this;
        }

        PolygonList& PolygonList::_xor_with_this(const PolygonList& rhs)
        {
            ClipperLib::Clipper clipper;
            ClipperLib::Paths paths;
            clipper.AddPaths(*this, ClipperLib::ptSubject, true);
            clipper.AddPaths(rhs, ClipperLib::ptClip, true);
            clipper.Execute(ClipperLib::ctXor, paths, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
            load(paths);
            return *this;
        }

        PolygonList::operator ClipperLib::Paths() const
        {
            ClipperLib::Paths paths;
            for (const Polygon& polygon : m_polygons)
            {
                paths.push_back(polygon);
            }
            return paths;
        }

        PolygonList::PolygonList(const ClipperLib::Paths& paths)
        {
            load(paths);
        }

        void PolygonList::load(const ClipperLib::Paths& paths)
        {
            m_polygons.clear();
            for (const ClipperLib::Path& path : paths)
            {
                m_polygons.push_back(Polygon(path));
            }
        }

        void from_json(const nlohmann::json& j, PolygonList& polygons)
        {
            for (const Polygon& polygon : j)
            {
                polygons.m_polygons.push_back(polygon);
            }
        }

        void to_json(nlohmann::json& j, const PolygonList& polygons)
        {
            j = polygons.m_polygons;
        }
    } // namespace geometry
} // namespace rip
