#include <teb_planner/line_obstacle.hpp>

#include <geometry/geometry_utils.hpp>

namespace rip
{
    namespace navigation
    {
        LineObstacle::LineObstacle(const geometry::Point& start, const geometry::Point& end)
            : m_start(start)
            , m_end(end)
        {}

        geometry::Point LineObstacle::start() const
        {
            return m_start;
        }

        geometry::Point LineObstacle::end() const
        {
            return m_end;
        }

        geometry::Point LineObstacle::centroid() const
        {
            return m_start + (m_end - m_start) / 2.0;
        }

        bool LineObstacle::collision(const geometry::Point& point, const units::Distance& minimum_distance) const
        {
            return minimumDistance(point) < minimum_distance;
        }

        bool LineObstacle::lineIntersection(const geometry::Point& start, const geometry::Point& end, const units::Distance& minimum_distance) const
        {
            geometry::Point intersection;
            return geometry::utils::lineIntersection(start, end, m_start, m_end, intersection);
        }

        units::Distance LineObstacle::minimumDistance(const geometry::Point& point) const
        {
            return geometry::utils::pointToSegment(point, m_start, m_end);
        }

        units::Distance LineObstacle::minimumDistance(const geometry::Point& start, const geometry::Point& end) const
        {
            return geometry::utils::segmentToSegment(start, end, m_start, m_end);
        }

        units::Distance LineObstacle::minimumDistance(const geometry::Polygon& polygon) const
        {
            return geometry::utils::segmentToPolygon(m_start, m_end, polygon);
        }

        geometry::Point LineObstacle::closestPoint(const geometry::Point& point) const
        {
            return geometry::utils::closestPointOnSegment(point, m_start, m_end);
        }

        units::Distance LineObstacle::minimumSpatioTemporalDistance(const geometry::Point& point, const units::Time& t) const
        {

        }

        units::Distance LineObstacle::minimumSpatioTemporalDistance(const geometry::Point& start, const geometry::Point& end, const units::Time& t) const
        {

        }

        units::Distance LineObstacle::minimumSpatioTemporalDistance(const geometry::Polygon& polygon, const units::Time& t) const
        {

        }

    }
}
