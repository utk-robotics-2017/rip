#include <teb_planner/point_robot_footprint_model.hpp>

namespace rip
{
    namespace navigation
    {

        rip::units::Distance rip::navigation::PointRobotFootprintModel::distance(const rip::navigation::Pose& current_pose, std::shared_ptr<Obstacle> obstacle) const
        {
            return obstacle->minimumDistance(current_pose.position());
        }

        units::Distance PointRobotFootprintModel::estimateSpatioTemporalDistance(const Pose& current_pose, std::shared_ptr<Obstacle> obstacle, const units::Time& t) const
        {
            return obstacle->minimumSpatioTemporalDistance(current_pose.position(), t);
        }

        units::Distance PointRobotFootprintModel::inscribedRadius() const
        {
            return 0.0;
        }

    }
}
