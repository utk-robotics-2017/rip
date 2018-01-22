#ifndef POINT_ROBOT_FOOTPRINT_MODEL_HPP
#define POINT_ROBOT_FOOTPRINT_MODEL_HPP

#include "robot_footprint_model.hpp"

namespace rip
{
    namespace navigation
    {
        class PointRobotFootprintModel : public RobotFootprintModel
        {
        public:
            /**
             * Default Constructor
             */
            PointRobotFootprintModel() = default;

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
             * Returns the inscribed radius of the footprint of the model
             */
            virtual units::Distance inscribedRadius() const override;
        };
    }
}

#endif // POINT_ROBOT_FOOTPRINT_MODEL_HPP
