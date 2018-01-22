#include <teb_planner/edge_obstacle.hpp>

#include <teb_planner/penalties.hpp>

namespace rip
{
    namespace navigation
    {

        EdgeObstacle::EdgeObstacle()
        {
            _measurement = nullptr;
        }

        void EdgeObstacle::computeError()
        {
            const VertexPose* bandpt = static_cast<VertexPose*>(_vertices[0]);

            units::Distance dist = m_robot_model->distance(bandpt->pose(), _measurement);

            _error[0] = penaltyBoundFromBelow(dist, m_config->obstacles.min_obstacle_distance, m_config->optimization.penalty_epsilon * units::in)();
        }

        void EdgeObstacle::setParameters(std::shared_ptr<TebConfig> config, std::shared_ptr<RobotFootprintModel> robot_model, std::shared_ptr<Obstacle> obstacle)
        {
            m_config = config;
            m_robot_model = robot_model;
            _measurement = obstacle;
        }

        EdgeDynamicObstacle::EdgeDynamicObstacle()
            : m_t(0)
        {}

        EdgeDynamicObstacle::EdgeDynamicObstacle(const units::Time& t)
            : m_t(t)
        {}

        void EdgeDynamicObstacle::computeError()
        {
            const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);
            units::Distance dist = m_robot_model->estimateSpatioTemporalDistance(bandpt->pose(), _measurement, m_t);
            _error[0] = penaltyBoundFromBelow(dist, m_config->obstacles.min_obstacle_distance, m_config->optimization.penalty_epsilon * units::in)();
            _error[1] = penaltyBoundFromBelow(dist, m_config->obstacles.dynamic_obstacle_inflation_distance, m_config->optimization.penalty_epsilon * units::in)();
        }

        void EdgeDynamicObstacle::setParameters(std::shared_ptr<TebConfig> config, std::shared_ptr<RobotFootprintModel> robot_model, std::shared_ptr<Obstacle> obstacle)
        {
            m_config = config;
            m_robot_model = robot_model;
            _measurement = obstacle;
        }

        EdgeInflatedObstacle::EdgeInflatedObstacle()
        {
            _measurement = nullptr;
        }

        void EdgeInflatedObstacle::computeError()
        {
            const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);

            units::Distance dist = m_robot_model->distance(bandpt->pose(), _measurement);

            _error[0] = penaltyBoundFromBelow(dist, m_config->obstacles.min_obstacle_distance, m_config->optimization.penalty_epsilon * units::in)();
            _error[1] = penaltyBoundFromBelow(dist, m_config->obstacles.inflation_distance, m_config->optimization.penalty_epsilon * units::in)();
        }

        void EdgeInflatedObstacle::setParameters(std::shared_ptr<TebConfig> config, std::shared_ptr<RobotFootprintModel> robot_model, std::shared_ptr<Obstacle> obstacle)
        {
            m_config = config;
            m_robot_model = robot_model;
            _measurement = obstacle;
        }

    }
}
