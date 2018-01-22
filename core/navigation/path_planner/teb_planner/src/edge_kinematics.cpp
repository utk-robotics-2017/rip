#include <teb_planner/edge_kinematics.hpp>

#include <geometry/point.hpp>

#include <teb_planner/penalties.hpp>

namespace rip
{
    namespace navigation
    {

        EdgeKinematics::EdgeKinematics()
        {
        }

        void EdgeKinematics::computeError()
        {
            const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
            const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);

            geometry::Point diff = pose2->position() - pose1->position();

            _error[0] = (fabs( units::cos(pose1->theta()) + units::cos(pose2->theta()) ) * diff.y() - (units::sin(pose1->theta()) + units::sin(pose2->theta())) * diff.x())();

            geometry::Point angle_vec(units::cos(pose1->theta()), units::sin(pose1->theta()));
            _error[1] = penaltyBoundFromBelow(diff.dot(angle_vec), 0.0, 0.0);
        }

    }
}
