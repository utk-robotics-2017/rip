#ifndef EDGE_WAYPOINT_HPP
#define EDGE_WAYPOINT_HPP

#include <eigen3/Eigen/Core>
#include <geometry/point.hpp>

#include "vertex_pose.hpp"
#include "teb_config.hpp"

namespace rip
{
    namespace navigation
    {
        /**
         * Edge defining the cost function for pushing the robot towards a waypoint
         */
        class EdgeWaypoint : public BaseUnaryEdge<1, geometry::Point, VertexPose>
        {
        public:
            EdgeWaypoint()
            {
            }

            void computeError()
            {
                const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);
                _error[0] = bandpt->position().distance(_measurement)();
            }

            void setParameters(std::shared_ptr<TebConfig> config, const geometry::Point& point)
            {
                m_config = config;
                _measurement = point;
            }

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };
    }
}

#endif // EDGE_WAYPOINT_HPP
