#ifndef EDGE_KINEMATICS_HPP
#define EDGE_KINEMATICS_HPP

#include <eigen3/Eigen/Core>

#include "edge.hpp"
#include "vertex_pose.hpp"

namespace rip
{
    namespace navigation
    {
        class EdgeKinematics : public BaseBinaryEdge<2, double, VertexPose, VertexPose>
        {
        public:
            EdgeKinematics();

            void computeError() override;

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };
    }
}

#endif // EDGE_KINEMATICS_HPP
