#ifndef GEOMETRY_UTILS_HPP
#define GEOMETRY_UTILS_HPP

#include <geometry/point.hpp>
#include <geometry/polygon.hpp>

namespace rip
{
    namespace geometry
    {
        namespace utils
        {
            /**             *             *
             * @ref http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
             */
            bool lineIntersection(const Point& s1, const Point& e1, const Point& s2, const Point& e2, Point& intersection);

            units::Distance pointToSegment(const Point& p, const Point& start, const Point& end);

            units::Distance segmentToSegment(const Point& s1, const Point& e1, const Point& s2, const Point& e2);

            units::Distance pointToPolygon(const Point& p, const Polygon& polygon);

            units::Distance segmentToPolygon(const Point& start, const Point& end, const Polygon& polygon);

            Point closestPointOnSegment(const Point& p, const Point& start, const Point& end);
        }
    }
}

#endif // GEOMETRY_UTILS_HPP
