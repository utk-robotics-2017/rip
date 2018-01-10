#ifndef POINT_ROBOT_FOOTPRINT_MODEL_HPP
#define POINT_ROBOT_FOOTPRINT_MODEL_HPP

namespace rip
{
    namespace navigation
    {
        class PointRobotFootprintModel : public RobotFootprintModelBase
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
            virtual units::Distance distance(const Pose& current_pose, std::shared_ptr<ObstacleBase> obstacle) const override
            {
                return obstacle->minimumDistance(current_pose.position());
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
                return obstacle->minimumSpatioTemporalDistance(current_pose.position(), t);
            }

            /**
             * Returns the inscribed radius of the footprint of the model
             */
            virtual units::Distance inscribedRadius() const override
            {
                return 0.0;
            }
        };
    }
}

#endif // POINT_ROBOT_FOOTPRINT_MODEL_HPP