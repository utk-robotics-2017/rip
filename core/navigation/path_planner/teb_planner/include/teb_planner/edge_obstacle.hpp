#ifndef EDGE_OBSTACLE_HPP
#define EDGE_OBSTACLE_HPP

#include <memory>

#include <eigen3/Eigen/Core>

#include "edge.hpp"
#include "obstacle.hpp"
#include "vertex_pose.hpp"
#include "robot_footprint_model.hpp"
#include "teb_config.hpp"

namespace rip
{
    namespace navigation
    {
        /**
         * Edge defining the cost function for keeping a minimum
         * distance from obstacles
         */
        class EdgeObstacle : public BaseUnaryEdge<1, std::shared_ptr<Obstacle>, VertexPose>
        {
        public:
            EdgeObstacle();

            void computeError();

            void setParameters(std::shared_ptr<TebConfig> config, std::shared_ptr<RobotFootprintModel> robot_model, std::shared_ptr<Obstacle> obstacle);

        protected:
            std::shared_ptr<RobotFootprintModel> m_robot_model;
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

        class EdgeDynamicObstacle : public BaseUnaryEdge<2, std::shared_ptr<Obstacle>, VertexPose>
        {
        public:
            EdgeDynamicObstacle();

            EdgeDynamicObstacle(const units::Time& t);

            void computeError() override;

            void setParameters(std::shared_ptr<TebConfig> config, std::shared_ptr<RobotFootprintModel> robot_model, std::shared_ptr<Obstacle> obstacle);
        protected:
            std::shared_ptr<RobotFootprintModel> m_robot_model;
            units::Time m_t;

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

        /**
         * Edge defining the cost function for keeping a minimum distance
         * from an inflated obstacle
         */
        class EdgeInflatedObstacle : public BaseUnaryEdge<2, std::shared_ptr<Obstacle>, VertexPose>
        {
        public:
            EdgeInflatedObstacle();

            void computeError() override;

            void setParameters(std::shared_ptr<TebConfig> config, std::shared_ptr<RobotFootprintModel> robot_model, std::shared_ptr<Obstacle> obstacle);
        private:
            std::shared_ptr<RobotFootprintModel> m_robot_model;
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };
    }
}

#endif // EDGE_OBSTACLE_HPP
