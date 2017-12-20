#include "polygon.hpp"

namespace rip
{
    namespace geometry
    {
        Polygon::Polygon(const std::vector< Point >& points)
            : m_points(points)
        {
        }

        bool Polygon::inside(const Point& point) const
        {
            int result = ClipperLib::PointInPolygon(point, *this);
            // -1 is on the border
            return result || result == -1;
        }

        Distance Polygon::perimeter() const
        {
            Distance rv = 0;
            Point prev = m_points[m_points.size() - 1];
            for (int i = 0; i < m_points.size(); i++)
            {
                Point current = m_points[i];
                rv += current.distance(prev)();
            }
            return rv;
        }

        Area Polygon::area() const
        {
            return ClipperLib::Area(*this);
        }

        Polygon::Orientation Polygon::orientation() const
        {
            return ClipperLib::Orientation(*this) ? Orientation::kClockwise : Orientation::kCounterClockwise;
        }

        bool Polygon::exterior() const
        {
            return ClipperLib::Orientation(*this);
        }

        bool Polygon::hole() const
        {
            return !exterior();
        }

        Distance Polygon::minX() const
        {
            Distance rv = std::numeric_limits<double>::max();
            for (const Point& point : m_points)
            {
                if (point.x() < rv)
                {
                    rv = point.x();
                }
            }
            return rv;
        }

        Distance Polygon::maxX() const
        {
            Distance rv = std::numeric_limits<double>::min();
            for (const Point& point : m_points)
            {
                if (point.x() > rv)
                {
                    rv = point.x();
                }
            }
            return rv;
        }

        Distance Polygon::minY() const
        {
            Distance rv = std::numeric_limits<double>::max();
            for (const Point& point : m_points)
            {
                if (point.y() < rv)
                {
                    rv = point.y();
                }
            }
            return rv;
        }

        Distance Polygon::maxY() const
        {
            Distance rv = std::numeric_limits<double>::min();
            for (const Point& point : m_points)
            {
                if (point.y() > rv)
                {
                    rv = point.y();
                }
            }
            return rv;
        }

        Rectangle Polygon::boundingBox() const
        {
            Point top_left(std::numeric_limits<double>::max(), std::numeric_limits<double>::min());
            Point bottom_right(std::numeric_limits<double>::min(), std::numeric_limits<double>::max());
            for (const Point& point : m_points)
            {
                if (point.y() > top_left.y())
                {
                    top_left.y(point.y());
                }

                if (point.x() < top_left.x())
                {
                    top_left.x(point.x());
                }

                if (point.y() < bottom_right.y())
                {
                    bottom_right.y(point.y());
                }

                if (point.x() > bottom_right.x())
                {
                    bottom_right.x(point.x());
                }
            }
            return Rectangle(top_left, bottom_right);
        }

        PolygonList Polygon::offset(const Distance& rhs) const
        {
            ClipperLib::Paths paths;
            ClipperLib::ClipperOffset clipper;
            clipper.AddPath(*this, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
            clipper.Execute(paths, rhs());
            return PolygonList(paths);
        }

        std::vector<Point>::const_iterator Polygon::begin() const
        {
            return m_points.cbegin();
        }

        std::vector<Point>::const_iterator Polygon::end() const
        {
            return m_points.cend();
        }

        std::vector<Point>::iterator Polygon::begin()
        {
            return m_points.begin();
        }

        std::vector<Point>::iterator Polygon::end()
        {
            return m_points.end();
        }

        Point Polygon::operator [](int index) const
        {
            return m_points[index];
        }

        size_t Polygon::size() const
        {
            return m_points.size();
        }

        PolygonList Polygon::operator +(const Polygon& rhs) const
        {
            return _add(rhs);
        }

        PolygonList Polygon::operator +(const PolygonList& rhs) const
        {
            return _add(rhs);
        }

        PolygonList Polygon::operator -(const Polygon& rhs) const
        {
            return _subtract(rhs);
        }

        PolygonList Polygon::operator -(const PolygonList& rhs) const
        {
            return _subtract(rhs);
        }

        PolygonList Polygon::operator &(const Polygon& rhs) const
        {
            return _intersection(rhs);
        }

        PolygonList Polygon::operator &(const PolygonList& rhs) const
        {
            return _intersection(rhs);
        }

        PolygonList Polygon::operator |(const Polygon& rhs) const
        {
            return _add(rhs);
        }

        PolygonList Polygon::operator |(const PolygonList& rhs) const
        {
            return _add(rhs);
        }

        PolygonList Polygon::operator ^(const Polygon& rhs) const
        {
            return _xor(rhs);
        }

        PolygonList Polygon::operator ^(const PolygonList& rhs) const
        {
            return _xor(rhs);
        }

        PolygonList Polygon::_add(const Polygon& rhs) const
        {
            ClipperLib::Paths paths;
            ClipperLib::Clipper clipper;
            clipper.AddPath(*this, ClipperLib::ptSubject, true);
            clipper.AddPath(rhs, ClipperLib::ptSubject, true);
            clipper.Execute(ClipperLib::ctUnion, paths, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
            return PolygonList(paths);
        }

        PolygonList Polygon::_add(const PolygonList& rhs) const
        {
            return rhs + *this;
        }

        PolygonList Polygon::_subtract(const Polygon& rhs) const
        {
            ClipperLib::Paths paths;
            ClipperLib::Clipper clipper;
            clipper.AddPath(*this, ClipperLib::ptSubject, true);
            clipper.AddPath(rhs, ClipperLib::ptClip, true);
            clipper.Execute(ClipperLib::ctDifference, paths, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
            return PolygonList(paths);
        }

        PolygonList Polygon::_subtract(const PolygonList& rhs) const
        {
            ClipperLib::Paths paths;
            ClipperLib::Clipper clipper;
            clipper.AddPath(*this, ClipperLib::ptSubject, true);
            clipper.AddPaths(rhs, ClipperLib::ptClip, true);
            clipper.Execute(ClipperLib::ctDifference, paths, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
            return PolygonList(paths);
        }

        PolygonList Polygon::_intersection(const Polygon& rhs) const
        {
            ClipperLib::Paths paths;
            ClipperLib::Clipper clipper;
            clipper.AddPath(*this, ClipperLib::ptSubject, true);
            clipper.AddPath(rhs, ClipperLib::ptClip, true);
            clipper.Execute(ClipperLib::ctIntersection, paths, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
            return PolygonList(paths);
        }

        PolygonList Polygon::_intersection(const PolygonList& rhs) const
        {
            return rhs & *this;
        }

        PolygonList Polygon::_xor(const Polygon& rhs) const
        {
            ClipperLib::Paths paths;
            ClipperLib::Clipper clipper;
            clipper.AddPath(*this, ClipperLib::ptSubject, true);
            clipper.AddPath(rhs, ClipperLib::ptClip, true);
            clipper.Execute(ClipperLib::ctXor, paths, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
            return PolygonList(paths);
        }

        PolygonList Polygon::_xor(const PolygonList& rhs) const
        {
            return rhs ^ *this;
        }

        Polygon::operator ClipperLib::Path() const
        {
            ClipperLib::Path path;
            for (Point point : m_points)
            {
                path.push_back(point);
            }
            return path;
        }

        Polygon::Polygon(const ClipperLib::Path& path)
        {
            for (const ClipperLib::IntPoint& point : path)
            {
                m_points.emplace_back(point);
            }
        }

        void from_json(const nlohmann::json& j, Polygon& polygon)
        {
            for (const Point& point : j)
            {
                polygon.m_points.push_back(point);
            }
        }

        void to_json(nlohmann::json& j, const Polygon& polygon)
        {
            j = polygon.m_points;
        }
    }
}