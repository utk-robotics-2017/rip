#include <teb_planner/polygon_obstacle.hpp>

namespace rip
{
    namespace navigation
    {

        PolygonObstacle::PolygonObstacle(const geometry::Polygon& polygon)
            : m_polygon(polygon)
        {}

        geometry::Polygon PolygonObstacle::polygon() const
        {
            return m_polygon;
        }

        void PolygonObstacle::setPolygon(const geometry::Polygon& polygon)
        {
            m_polygon = polygon;
        }

        void PolygonObstacle::setPoint(unsigned int index, const geometry::Point& point)
        {
            assert(index < m_polygon.size());
            m_polygon[index] = point;
        }

        geometry::Point PolygonObstacle::centroid() const
        {
            return m_polygon.centroid();
        }

        void PolygonObstacle::setCentroid(const geometry::Point& new_centroid)
        {
            geometry::Point offset = new_centroid - centroid();

            for(geometry::Point& p : m_polygon)
            {
                p += offset;
            }
        }

        bool PolygonObstacle::collision(const geometry::Point& point, const units::Distance& minimum_distance) const
        {
            return m_polygon.offset(minimum_distance).inside(point);
        }

        bool PolygonObstacle::lineIntersection(const geometry::Point& start, const geometry::Point& end, const units::Distance& minimum_distance) const
        {

        }

        units::Distance PolygonObstacle::minimumDistance(const geometry::Point& point) const
        {

        }

        units::Distance PolygonObstacle::minimumDistance(const geometry::Point& start, const geometry::Point& end) const
        {

        }

        units::Distance PolygonObstacle::minimumDistance(const geometry::Polygon& polygon) const
        {

        }

        geometry::Point PolygonObstacle::closestPoint(const geometry::Point& point) const
        {

        }

        units::Distance PolygonObstacle::minimumSpatioTemporalDistance(const geometry::Point& point, const units::Time& t) const
        {

        }

        units::Distance PolygonObstacle::minimumSpatioTemporalDistance(const geometry::Point& start, const geometry::Point& end, const units::Time& t) const
        {

        }

        units::Distance PolygonObstacle::minimumSpatioTemporalDistance(const geometry::Polygon& polygon, const units::Time& t) const
        {

        }

    }
}
