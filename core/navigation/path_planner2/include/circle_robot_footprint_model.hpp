#ifndef CIRCLE_ROBOT_FOOTPRINT_MODEL_HPP
#define CIRCLE_ROBOT_FOOTPRINT_MODEL_HPP

namespace rip
{
    namespace navigation
    {
        /**
         * Class the defines a robot of circular shape
         */
        class CircleRobotFootprintModel : public RobotFootprintModelBase
        {
        public:
            /**
             * Constructor
             */
            CircleRobotFootprintModel(const units::Distance& radius)
                : m_radius(radius)
            {}

            /**
             * Returns the distance between the robot and an obstacle
             * @param  current_pose Current robot pose
             * @param  obstacles    Obstacle
             * @return              Euclidean distance from the robot to the obstacle
             */
            virtual units::Distance distance(const Pose& current_pose, std::shared_ptr<ObstacleBase> obstacle) const override
            {
                return obstacle->minimumDistance(current_pose.position()) - m_radius;
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
                return obstacle->minimumSpatioTemporalDistance(current_pose.position(), t) - m_radius;
            }

            /**
             * Returns the inscribed radius of the footprint model
             */
            virtual units::Distance inscribedRadius() const override
            {
                return m_radius;
            }
        private:
            units::Distance m_radius;
        };
    }
}


#endif // CIRCLE_ROBOT_FOOTPRINT_MODEL_HPP
