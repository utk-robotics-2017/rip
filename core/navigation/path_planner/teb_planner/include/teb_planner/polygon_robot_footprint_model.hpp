#ifndef POLYGON_ROBOT_FOOTPRINT_MODEL_HPP
#define POLYGON_ROBOT_FOOTPRINT_MODEL_HPP

#include "robot_footprint_model.hpp"

namespace rip
{
    namespace navigation
    {
        /**
         * Class that approximates the robot with a closed polygon
         */
        class PolygonRobotFootprintModel : public RobotFootprintModel
        {
        public:
            /**
             * Constructor
             */
            PolygonRobotFootprintModel(const geometry::Polygon& polygon);

            geometry::Polygon polygon( const geometry::Point& position, const geometry::Angle& theta) const;

            geometry::Polygon polygon( const Pose& pose) const;

            /**
             * Returns the distance between the robot and an obstacle
             * @param  current_pose Current robot pose
             * @param  obstacles    Obstacle
             * @return              Euclidean distance from the robot to the obstacle
             */
            virtual units::Distance distance(const Pose& current_pose, std::shared_ptr<Obstacle> obstacle) const override;

            /**
             * Estimate the distance between the robot and the predicted location of an obstacle at time t
             * @param  current_pose Current robot pose
             * @param  obstacle     Obstacle
             * @param  t            Time for which the predicted distance to the obstacle is calculated
             * @return              Euclidean distance from the robot to the obstacle
             */
            virtual units::Distance estimateSpatioTemporalDistance(const Pose& current_pose, std::shared_ptr<Obstacle> obstacle, const units::Time& t) const override;

            /**
             * Returns the inscribed radius of the footprint model
             */
            virtual units::Distance inscribedRadius() const override;
        protected:
            /**
             * Transforms a polygon to the world frame
             * @param  pose Current Robot Pose
             * @return      The polygon transformed to the world frame
             */
            geometry::Polygon transformToWorld(const Pose& pose) const;


            geometry::Polygon m_polygon;
        };
    }
}

#endif // POLYGON_ROBOT_FOOTPRINT_MODEL_HPP
