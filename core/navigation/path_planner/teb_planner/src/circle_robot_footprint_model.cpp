#include <teb_planner/circle_robot_footprint_model.hpp>

namespace rip
{
    namespace navigation
   {
        CircleRobotFootprintModel::CircleRobotFootprintModel(const units::Distance& radius)
            : m_radius(radius)
        {}

        units::Distance CircleRobotFootprintModel::distance(const Pose& current_pose, std::shared_ptr<Obstacle> obstacle) const
        {
            return obstacle->minimumDistance(current_pose.position()) - m_radius;
        }

        units::Distance CircleRobotFootprintModel::estimateSpatioTemporalDistance(const Pose& current_pose, std::shared_ptr<Obstacle> obstacle, const units::Time& t) const
        {
            return obstacle->minimumSpatioTemporalDistance(current_pose.position(), t) - m_radius;
        }

        units::Distance CircleRobotFootprintModel::inscribedRadius() const
        {
            return m_radius;
        }
    }
}
