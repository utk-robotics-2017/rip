#include <teb_planner/polygon_robot_footprint_model.hpp>

#include <geometry/geometry_utils.hpp>

namespace rip
{
    namespace navigation
    {

        PolygonRobotFootprintModel::PolygonRobotFootprintModel(const geometry::Polygon& polygon)
            : m_polygon(polygon)
        {}

        geometry::Polygon PolygonRobotFootprintModel::polygon(const geometry::Point& position, const geometry::Angle& theta) const
        {
            return polygon(Pose(position, theta));
        }

        geometry::Polygon PolygonRobotFootprintModel::polygon(const Pose& pose) const
        {
            return transformToWorld(pose);
        }

        units::Distance PolygonRobotFootprintModel::distance(const Pose& current_pose, std::shared_ptr<Obstacle> obstacle) const
        {
            geometry::Polygon transformed_polygon = transformToWorld(current_pose);
            return obstacle->minimumDistance(transformed_polygon);
        }

        units::Distance PolygonRobotFootprintModel::estimateSpatioTemporalDistance(const Pose& current_pose, std::shared_ptr<Obstacle> obstacle, const units::Time& t) const
        {
            geometry::Polygon transformed_polygon = transformToWorld(current_pose);
            return obstacle->minimumSpatioTemporalDistance(transformed_polygon, t);
        }

        units::Distance PolygonRobotFootprintModel::inscribedRadius() const
        {
            units::Distance minimum_distance = std::numeric_limits<double>::max();
            geometry::Point center;

            if (m_polygon.size() <= 2)
            {
                return 0.0;
            }

            for (int i = 0, end = m_polygon.size() - 1; i < end; i++)
            {
                units::Distance vertex_distance = m_polygon[i].magnitude();
                units::Distance edge_distance = geometry::utils::pointToSegment(center, m_polygon[i], m_polygon[i + 1]);
                minimum_distance = units::min(minimum_distance, units::min(vertex_distance, edge_distance));
            }

            units::Distance vertex_distance = m_polygon.back().magnitude();
            units::Distance edge_distance = geometry::utils::pointToSegment(center, m_polygon.back(), m_polygon.front());
            return units::min(minimum_distance, units::min(vertex_distance, edge_distance));
        }

        geometry::Polygon PolygonRobotFootprintModel::transformToWorld(const Pose& pose) const
        {
            double cos_th = units::cos(pose.theta());
            double sin_th = units::sin(pose.theta());

            geometry::Polygon out;
            for (size_t i = 0, end = m_polygon.size(); i < end; i++)
            {
                geometry::Point p(pose.x() + cos_th * m_polygon[i].x() - sin_th * m_polygon[i].y(),
                                  pose.y() + sin_th * m_polygon[i].x() + cos_th * m_polygon[i].y());
                out += p;
            }

            return out;
        }

    }
}
