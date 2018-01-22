#include <teb_planner/edge_velocity.hpp>
#include <teb_planner/penalties.hpp>

namespace rip
{
    namespace navigation
    {
        /**
        * Calculate a fast approximation of a sigmoid function
        */
        inline double fast_sigmoid(double x)
        {
            return x / (1 + fabs(x));
        }

        EdgeVelocity::EdgeVelocity()
        {
            resize(3);
        }

        void EdgeVelocity::computeError()
        {
            const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
            const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
            const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);

            const geometry::Point diff = pose2->position() - pose1->position();

            units::Distance dist = diff.magnitude();
            const units::Angle angle_diff = g2o::normalize_theta(pose2->theta().to(units::rad) - pose1->theta().to(units::rad)) * units::rad;
            if (m_config->trajectory.exact_arc_length && angle_diff != 0)
            {
                const units::Distance radius = dist / (2.0 * units::sin(angle_diff / 2.0));
                dist = units::abs(angle_diff.to(units::rad) * radius);
            }

            units::Velocity vel = dist / dt->dt();
            vel *= fast_sigmoid((100 * (diff.x() * units::cos(pose1->theta()) + diff.y() * units::sin(pose1->theta())))());

            const units::AngularVelocity omega = angle_diff / dt->dt();

            _error[0] = penaltyBoundToInterval(vel, -m_config->robot.max_velocity_x_backwards, m_config->robot.max_velocity_x, m_config->optimization.penalty_epsilon * units::in / units::s)();
            _error[1] = penaltyBoundToInterval(omega, m_config->robot.max_velocity_theta, m_config->optimization.penalty_epsilon * units::rad / units::s)();
        }

        EdgeVelocityHolonomic::EdgeVelocityHolonomic()
        {
            resize(3);
        }

        void EdgeVelocityHolonomic::computeError()
        {
            const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
            const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
            const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);

            geometry::Point diff = pose2->position() - pose1->position();

            double cos_theta1 = units::cos(pose1->theta());
            double sin_thetat1 = units::sin(pose1->theta());

            units::Distance r_dx = cos_theta1 * diff.x() + sin_thetat1 * diff.y();
            units::Distance r_dy = -sin_thetat1 * diff.x() + cos_theta1 * diff.y();

            units::Velocity vx = r_dx / dt->dt();
            units::Velocity vy = r_dy / dt->dt();
            units::AngularVelocity omega = g2o::normalize_theta(pose2->theta().to(units::rad) - pose1->theta().to(units::rad)) * units::rad / dt->dt();

            _error[0] = penaltyBoundToInterval(vx, -m_config->robot.max_velocity_x_backwards, m_config->robot.max_velocity_x, m_config->optimization.penalty_epsilon * units::in / units::s)();
            _error[1] = penaltyBoundToInterval(vy, m_config->robot.max_velocity_y, m_config->optimization.penalty_epsilon * units::in / units::s)();
            _error[2] = penaltyBoundToInterval(omega, m_config->robot.max_velocity_theta, m_config->optimization.penalty_epsilon * units::rad / units::s)();
        }


    }
}
