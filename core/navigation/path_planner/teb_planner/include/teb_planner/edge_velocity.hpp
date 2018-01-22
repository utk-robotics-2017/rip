#ifndef EDGE_VELOCITY_HPP
#define EDGE_VELOCITY_HPP

#include <geometry/point.hpp>

#include "edge.hpp"
#include "vertex_pose.hpp"
#include "vertex_time_diff.hpp"

namespace rip
{
    namespace navigation
    {
        /**
         * Edge defining the cost function for limiting translational
         * and rotational velocity
         */
        class EdgeVelocity : public BaseMultiEdge<2, double>
        {
        public:
            /**
             * Constructor
             */
            EdgeVelocity();

            /**
             * Cost function
             */
            void computeError();

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

        /**
         * Edge defining cost function for limiting translational and
         * rotational velocity for holonomic driving
         */
        class EdgeVelocityHolonomic : public BaseMultiEdge<3, double>
        {
        public:
            /**
             * Constructor
             */
            EdgeVelocityHolonomic();

            /**
             * Cost function
             */
            void computeError();

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };
    }
}

#endif // EDGE_VELOCITY_HPP
