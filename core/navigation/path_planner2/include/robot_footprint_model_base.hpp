#ifndef ROBOT_FOOTPRINT_MODEL_BASE_HPP
#define ROBOT_FOOTPRINT_MODEL_BASE_HPP

#include <memory>

#include <units.hpp>

#include "pose.hpp"
#include "obstacle_base.hpp"

namespace rip
{
    namespace navigation
    {
        /**
         * @class RobotFootprintModelBase
         * @brief Abstract class that defines the interface for robot
         * footprint/contour models
         *
         * The robot model class is currently used in optimization only, since
         * taking the navigation stack footprint into account might be
         * inefficient. The footprint is only used for checking feasibility.
         */
        class RobotFootprintModelBase
        {
        public:
            /**
             * Default Constructor
             */
            RobotFootprintModelBase() = default;

            /**
             * Default Destructor
             */
            virtual ~RobotFootprintModelBase() = default;

            /**
             * Returns the distance between the robot and an obstacle
             * @param  current_pose Current robot pose
             * @param  obstacles    Obstacle
             * @return              Euclidean distance from the robot to the obstacle
             */
            virtual units::Distance distance(const Pose& current_pose, std::shared_ptr<ObstacleBase> obstacles) const = 0;

            /**
             * Estimate the distance between the robot and the predicted location of an obstacle at time t
             * @param  current_pose Current robot pose
             * @param  obstacle     Obstacle
             * @param  t            Time for which the predicted distance to the obstacle is calculated
             * @return              Euclidean distance from the robot to the obstacle
             */
            virtual units::Distance estimateSpatioTemporalDistance(const Pose& current_pose, std::shared_ptr<ObstacleBase> obstacle, const units::Time& t) const = 0;

            /**
             * Returns the inscribed radius of the footprint model
             */
            virtual units::Distance inscribedRadius() const = 0;
        };
    }
}

#endif // ROBOT_FOOTPRINT_MODEL_BASE_HPP
