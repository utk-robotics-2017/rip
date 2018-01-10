#ifndef POLYGON_ROBOT_FOOTPRINT_MODEL_HPP
#define POLYGON_ROBOT_FOOTPRINT_MODEL_HPP

namespace rip
{
    namespace navigation
    {
        /**
         * Class that approximates the robot with a closed polygon
         */
        class PolygonRobotFootprintModel : public RobotFootprintModelBase
        {
        public:
            /**
             * Constructor
             */
            PolygonRobotFootprintModel(const Polygon& polygon)
                : m_polygon(polygon)
            {}

            /**
             * Returns the distance between the robot and an obstacle
             * @param  current_pose Current robot pose
             * @param  obstacles    Obstacle
             * @return              Euclidean distance from the robot to the obstacle
             */
            virtual units::Distance distance(const Pose& current_pose, std::shared_ptr<ObstacleBase> obstacle) const override
            {
                Polygon transformed_polygon = transformToWorld(current_pose);
                return obstacle->minimumDistance(transformed_polygon);
            }

            /**
             * Estimate the distance between the robot and the predicted location of an obstacle at time t
             * @param  current_pose Current robot pose
             * @param  obstacle     Obstacle
             * @param  t            Time for which the predicted distance to the obstacle is calculated
             * @return              Euclidean distance from the robot to the obstacle
             */
            virtual units::Distance estimateSpatioTemporalDistance(const Pose& current_pose, std::shared_ptr<ObstacleBase> obstacle, const units::Time& t) const override
            {
                Polygon transformed_polygon = transformToWorld(current_pose);
                return obstacle->minimumSpatioTemporalDistance(transformed_polygon);
            }

            /**
             * Returns the inscribed radius of the footprint model
             */
            virtual units::Distance inscribedCircle() const override
            {
                units::Distance minimum_distance = std::numeric_limits<double>::max();
                Point center;

                if (m_polygon.size() <= 2)
                {
                    return 0.0;
                }

                for (int i = 0, end = m_polygon.size() - 1; i < end; i++)
                {
                    units::Distance vertex_distance = m_polygon[i].distance();
                    units::Distance edge_distance = pointToSegment(center, m_polygon[i], m_polygon[i + 1]);
                    minimum_distance = units::min(minimum_distance, units::minimum_distance(vertex_distance, edge_distance));
                }

                units::Distance vertex_distance = m_polygon.back().distance();
                units::Distance edge_distance = pointToSegment(center, m_polygon.back(), m_polygon.front());
                return units::min(minimum_distance, units::min(vertex_distance, edge_distance));
            }
        protected:
            /**
             * Transforms a polygon to the world frame
             * @param  pose Current Robot Pose
             * @return      The polygon transformed to the world frame
             */
            Polygon transformToWorld(const Pose& pose) const
            {
                double cos_th = units::cos(pose.theta());
                double sin_th = units::sin(pose.theta());

                Polygon out;
                for (size_t i = 0, end = m_polygon.size(); i < end; i++)
                {
                    Point p(pose.x() + cos_th * m_polygon[i].x() - sin_th * m_polygon[i].y(),
                            pose.y() + sin_th * m_polygon[i].x() + cos_th * m_polygon[i].y());
                    out += p;
                }

                return out;
            }


            Polygon m_polygon;
        };
    }
}

#endif // POLYGON_ROBOT_FOOTPRINT_MODEL_HPP
