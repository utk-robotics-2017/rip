#ifndef EDGE_ACCELERATION_HPP
#define EDGE_ACCELERATION_HPP

#include <eigen3/Eigen/Core>

#include "edge.hpp"

#include "velocity_pose.hpp"

namespace rip
{
    namespace navigation
    {
        class EdgeAcceleration : public BaseMultiEdge<2, double>
        {
        public:
            EdgeAcceleration();

            void computeError() override;

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

        /**
         * Edge defining the cost function for limiting the translational
         * and rotational acceleration at the start of a trajectory
         */
        class EdgeAccelerationStart : public BaseMultiEdge<2, VelocityPose>
        {
        public:
            EdgeAccelerationStart();

            /**
             * Actual cost function
             */
            void computeError() override;

            void setStartVelocity(const VelocityPose& velocity_start);

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

        /**
         * Edge defining the cost function for limiting the translational
         * and rotational acceleration at the start of a trajectory
         */
        class EdgeAccelerationGoal : public BaseMultiEdge<2, VelocityPose>
        {
        public:
            EdgeAccelerationGoal();

            /**
             * Actual cost function
             */
            void computeError() override;

            void setGoalVelocity(const VelocityPose& velocity_goal);

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

        /**
         * Edge defining the cost function for limiting the translational
         * and rotational acceleration of an holonomic robot
         */
        class EdgeAccelerationHolonomic : public BaseMultiEdge<3, double>
        {
        public:
            EdgeAccelerationHolonomic();

            void computeError() override;

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

        /**
         * Edge defining the cost function for limiting the translational
         * and rotational acceleration of an holonomic robot
         */
        class EdgeAccelerationHolonomicStart : public BaseMultiEdge<3, VelocityPose>
        {
        public:
            EdgeAccelerationHolonomicStart();

            void computeError() override;

            void setStartVelocity(const VelocityPose& velocity);

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

        /**
         * Edge defining the cost function for limiting the translational
         * and rotational acceleration of an holonomic robot
         */
        class EdgeAccelerationHolonomicGoal : public BaseMultiEdge<3, VelocityPose>
        {
        public:
            EdgeAccelerationHolonomicGoal();

            void computeError() override;

            void setGoalVelocity(const VelocityPose& velocity);

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

    }
}

#endif // EDGE_ACCELERATION_HPP
