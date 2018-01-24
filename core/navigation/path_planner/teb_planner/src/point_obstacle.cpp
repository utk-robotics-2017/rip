#include <teb_planner/point_obstacle.hpp>

#include <geometry/geometry_utils.hpp>

namespace rip
{
    namespace navigation
    {

        PointObstacle::PointObstacle(const units::Distance& x, const units::Distance& y)
            : m_point(x, y)
        {
        }

        PointObstacle::PointObstacle(const geometry::Point& point)
            : m_point(point)
        {}

        geometry::Point PointObstacle::centroid() const
        {
            return m_point;
        }

        geometry::Point PointObstacle::position() const
        {
            return m_point;
        }

        void PointObstacle::setPosition(const geometry::Point &p)
        {
            m_point = p;
        }

        units::Distance PointObstacle::x() const
        {
            return m_point.x();
        }

        void PointObstacle::setX(const units::Distance& x)
        {
            m_point.setX(x);
        }

        units::Distance PointObstacle::y() const
        {
            return m_point.y();
        }

        void PointObstacle::setY(const units::Distance& y)
        {
            m_point.setY(y);
        }

        bool PointObstacle::collision(const geometry::Point& point, const units::Distance& minimum_distance) const
        {
            return point == m_point || minimumDistance(point) < minimum_distance;
        }

        bool PointObstacle::lineIntersection(const geometry::Point& start, const geometry::Point& end, const units::Distance& minimum_distance) const
        {
            // Distance Line - Circle
            // refer to http://www.spieleprogrammierer.de/wiki/2D-Kollisionserkennung#Kollision_Kreis-Strecke
            geometry::Point a = end - start;
            geometry::Point b = m_point - start;

            // Now find nearest point to circle v=x+a*t with t=a*b/(a*a) and bound to 0<=t<=1
            double t = a.dot(b) / a.dot(a);

            // t is bound since a isn't normalized
            if (t < 0)
            {
                t = 0;
            }
            else if (t > 1)
            {
                t = 1;
            }

            geometry::Point nearest = start + a * t;

            return collision(nearest, minimum_distance);
        }

        units::Distance PointObstacle::minimumDistance(const geometry::Point& point) const
        {
            return m_point.distance(point);
        }

        units::Distance PointObstacle::minimumDistance(const geometry::Point& start, const geometry::Point& end) const
        {
            return geometry::utils::pointToSegment(m_point, start, end);
        }

        units::Distance PointObstacle::minimumDistance(const geometry::Polygon& polygon) const
        {
            return geometry::utils::pointToPolygon(m_point, polygon);
        }

        geometry::Point PointObstacle::closestPoint(const geometry::Point& point) const
        {
            return m_point;
        }

        units::Distance PointObstacle::minimumSpatioTemporalDistance(const geometry::Point& point, const units::Time& t) const
        {

        }

        units::Distance PointObstacle::minimumSpatioTemporalDistance(const geometry::Point& start, const geometry::Point& end, const units::Time& t) const
        {

        }

        units::Distance PointObstacle::minimumSpatioTemporalDistance(const geometry::Polygon& polygon, const units::Time& t) const
        {

        }

    }
}
