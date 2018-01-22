#ifndef EDGE_TIME_OPTIMAL_HPP
#define EDGE_TIME_OPTIMAL_HPP

#include <eigen3/Eigen/Core>

#include "vertex_time_diff.hpp"

namespace rip
{
    namespace navigation
    {
        class EdgeTimeOptimal : public BaseUnaryEdge<1, double, VertexTimeDiff>
        {
        public:
            EdgeTimeOptimal()
            {
            }

            void computeError()
            {
                const VertexTimeDiff* timediff = static_cast<const VertexTimeDiff*>(_vertices[0]);
                _error[0] = timediff->dt()();
            }

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };
    }
}

#endif // EDGE_TIME_OPTIMAL_HPP
