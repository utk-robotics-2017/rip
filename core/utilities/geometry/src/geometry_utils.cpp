#include "geometry/geometry_utils.hpp"

namespace rip
{
    namespace geometry
    {
        namespace utils
        {
            bool lineIntersection(const Point& s1, const Point& e1, const Point& s2, const Point& e2, Point& intersection)
            {
                Point line1 = e1 - s1;
                Point line2 = e2 - s2;

                double denom = (line1.x() * line2.y() - line2.x() * line1.y())();
                if (denom == 0)
                {
                    return false;
                }
                Point aux = s1 - s2;

                double s_numer = (line1.x() * aux.y() - line1.y() * aux.x())();
                if (s_numer < 0 == denom > 0)
                {
                    return false;
                }

                double t_numer = (line2.x() * aux.y() - line2.y() * aux.x())();
                if (t_numer < 0 == denom > 0)
                {
                    return false;
                }

                if (s_numer > denom == denom > 0 || t_numer > denom == denom > 0)
                {
                    return false;
                }

                double t = t_numer / denom;
                intersection = s1 + t * line1;
                return true;
            }

            units::Distance pointToSegment(const Point& p, const Point& start, const Point& end)
            {
                return p.distance(closestPointOnSegment(p, start, end));
            }

            units::Distance segmentToSegment(const Point& s1, const Point& e1, const Point& s2, const Point& e2)
            {
                Point tmp;
                if (lineIntersection(s1, e1, s2, e2, tmp))
                {
                    return 0.0;
                }

                // check all 4 combinations
                std::array<units::Distance, 4> distances;

                distances[0] = pointToSegment(s1, s2, e2);
                distances[1] = pointToSegment(e1, s2, e2);
                distances[2] = pointToSegment(s2, s1, e1);
                distances[3] = pointToSegment(e2, s1, e1);

                return *std::min_element(distances.begin(), distances.end());
            }

            units::Distance pointToPolygon(const Point& p, const Polygon& polygon)
            {
                if (polygon.size() == 1)
                {
                    return p.distance(polygon.front());
                }

                units::Distance dist = std::numeric_limits<double>::max();
                for (int i = 0; i < polygon.size(); i++)
                {
                    units::Distance new_d = pointToSegment(p, polygon[i], polygon[i + 1]);
                    dist = units::min(dist, new_d);
                }

                if (polygon.size() > 2)
                {
                    units::Distance new_d = pointToSegment(p, polygon.back(), polygon.front());
                    dist = units::min(dist, new_d);
                }

                return dist;
            }

            units::Distance segmentToPolygon(const Point& start, const Point& end, const Polygon& polygon)
            {
                if (polygon.size() == 1)
                {
                    return pointToSegment(polygon.front(), start, end);
                }

                units::Distance dist = std::numeric_limits<double>::max();
                for (int i = 0; i < polygon.size(); i++)
                {
                    units::Distance new_d = segmentToSegment(start, end, polygon[i], polygon[i + 1]);
                    dist = units::min(dist, new_d);
                }

                if (polygon.size() > 2)
                {
                    units::Distance new_d = segmentToSegment(start, end, polygon.back(), polygon.front());
                    dist = units::min(dist, new_d);
                }

                return dist;
            }

            Point closestPointOnSegment(const Point& p, const Point& start, const Point& end)
            {
                Point diff = end - start;
                double sq_norm = pow(diff.magnitude()(), 2);

                if (sq_norm == 0)
                {
                    return start;
                }

                double u = ((p.x() - start.x()) * diff.x() + (p.y() - start.y()) * diff.y())() / sq_norm;

                if (u <= 0)
                {
                    return start;
                }
                if (u >= 1)
                {
                    return end;
                }

                return start + u * diff;
            }
        }
    }
}
