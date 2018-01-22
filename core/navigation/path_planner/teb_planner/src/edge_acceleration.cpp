#include "teb_planner/edge_acceleration.hpp"

#include <teb_planner/vertex_pose.hpp>
#include <teb_planner/vertex_time_diff.hpp>
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

        EdgeAcceleration::EdgeAcceleration()
        {
            resize(5);
        }

        void EdgeAcceleration::computeError()
        {
            const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
            const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
            const VertexPose* pose3 = static_cast<const VertexPose*>(_vertices[2]);
            const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[3]);
            const VertexTimeDiff* dt2 = static_cast<const VertexTimeDiff*>(_vertices[4]);

            // Velocity & Acceleration
            geometry::Point diff1 = pose2->position() - pose1->position();
            geometry::Point diff2 = pose3->position() - pose2->position();

            units::Distance dist1 = diff1.magnitude();
            units::Distance dist2 = diff2.magnitude();

            units::Angle angle_diff1 = g2o::normalize_theta(pose2->theta().to(units::rad) - pose1->theta().to(units::rad)) * units::rad;
            units::Angle angle_diff2 = g2o::normalize_theta(pose3->theta().to(units::rad) - pose2->theta().to(units::rad)) * units::rad;

            // Use exact arc length instead of
            if (m_config->trajectory.exact_arc_length)
            {
                if (angle_diff1 != 0)
                {
                    const units::Distance radius = dist1 / (2 * units::sin(angle_diff1 / 2.0));
                    dist1 = units::abs(angle_diff1.to(units::rad) * radius);
                }

                if (angle_diff2 != 0)
                {
                    const units::Distance radius = dist2 / (2 * units::sin(angle_diff2 / 2.0));
                    dist2 = units::abs(angle_diff2.to(units::rad) * radius);
                }
            }

            units::Velocity vel1 = dist1 / dt1->dt();
            units::Velocity vel2 = dist2 / dt2->dt();

            //Consider directions
            vel1 *= fast_sigmoid((100 * (diff1.x() * units::cos(pose1->theta()) + diff1.y() * units::sin(pose1->theta())))());
            vel2 *= fast_sigmoid((100 * (diff1.x() * units::cos(pose2->theta()) + diff1.y() * units::sin(pose2->theta())))());


            units::Acceleration linear_acceleration = (vel2 - vel1) * 2.0 / (dt1->dt() + dt2->dt());

            _error[0] = penaltyBoundToInterval<units::Acceleration>(linear_acceleration, m_config->robot.acceleration_limit_x, m_config->optimization.penalty_epsilon * units::in / units::s / units::s)();

            // Angular Acceleration
            units::AngularVelocity omega1 = angle_diff1 / dt1->dt();
            units::AngularVelocity omega2 = angle_diff2 / dt2->dt();
            units::AngularAcceleration rotational_acceleration = (omega2 - omega1) * 2.0 / (dt1->dt() + dt2->dt());

            _error[1] = penaltyBoundToInterval<units::AngularAcceleration>(rotational_acceleration, m_config->robot.acceleration_limit_theta, m_config->optimization.penalty_epsilon * units::rad / units::s / units::s)();
        }

        EdgeAccelerationStart::EdgeAccelerationStart()
        {
            resize(3);
        }

        void EdgeAccelerationStart::computeError()
        {
            const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
            const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
            const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);

            // Velocity and Acceleration
            const geometry::Point diff = pose2->position() - pose1->position();
            units::Distance dist = diff.magnitude();
            const units::Angle angle_diff = g2o::normalize_theta(pose2->theta().to(units::rad) - pose1->theta().to(units::rad)) * units::rad;
            if (m_config->trajectory.exact_arc_length && angle_diff != 0)
            {
                const units::Distance radius = dist / (2 * units::sin(angle_diff / 2.0));
                dist = units::abs(angle_diff.to(units::rad) * radius);
            }

            const units::Velocity vel1 = _measurement.x();
            units::Velocity vel2 = dist / dt->dt();

            // Consider the directions
            vel2 *= fast_sigmoid((100 * (diff.x() * units::cos(pose1->theta()) + diff.y() * units::sin(pose1->theta())))());

            const units::Acceleration linear_acceleration = (vel2 - vel1) / dt->dt();

            _error[0] = penaltyBoundToInterval<units::Acceleration>(linear_acceleration, m_config->robot.acceleration_limit_x, m_config->optimization.penalty_epsilon * units::in / units::s / units::s)();

            // Angular Acceleration
            const units::AngularVelocity omega1 = _measurement.omega();
            const units::AngularVelocity omega2 = angle_diff / dt->dt();
            const units::AngularAcceleration rotational_acceleration = (omega2 - omega1) / dt->dt();

            _error[1] = penaltyBoundToInterval<units::AngularAcceleration>(rotational_acceleration, m_config->robot.acceleration_limit_theta, m_config->optimization.penalty_epsilon * units::rad / units::s / units::s)();
        }

        void EdgeAccelerationStart::setStartVelocity(const VelocityPose& velocity_start)
        {
            _measurement = velocity_start;
        }

        EdgeAccelerationGoal::EdgeAccelerationGoal()
        {
            resize(3);
        }

        void EdgeAccelerationGoal::computeError()
        {
            const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
            const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
            const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);

            // Velocity and Acceleration
            const geometry::Point diff = pose2->position() - pose1->position();
            units::Distance dist = diff.magnitude();
            const units::Angle angle_diff = g2o::normalize_theta(pose2->theta().to(units::rad) - pose1->theta().to(units::rad)) * units::rad;
            if (m_config->trajectory.exact_arc_length && angle_diff != 0)
            {
                const units::Distance radius = dist / (2 * units::sin(angle_diff / 2.0));
                dist = units::abs(angle_diff.to(units::rad) * radius);
            }

            units::Velocity vel1 = dist / dt->dt();
            const units::Velocity vel2 = _measurement.x();

            // Consider the directions
            vel1 *= fast_sigmoid((100 * (diff.x() * units::cos(pose1->theta()) + diff.y() * units::sin(pose1->theta())))());

            const units::Acceleration linear_acceleration = (vel2 - vel1) / dt->dt();

            _error[0] = penaltyBoundToInterval(linear_acceleration, m_config->robot.acceleration_limit_x, m_config->optimization.penalty_epsilon * units::in / units::s / units::s)();

            // Angular Acceleration
            const units::AngularVelocity omega1 = _measurement.omega();
            const units::AngularVelocity omega2 = angle_diff / dt->dt();
            const units::AngularAcceleration rotational_acceleration = (omega2 - omega1) / dt->dt();

            _error[1] = penaltyBoundToInterval<units::AngularAcceleration>(rotational_acceleration, m_config->robot.acceleration_limit_theta, m_config->optimization.penalty_epsilon * units::rad / units::s / units::s)();
        }

        void EdgeAccelerationGoal::setGoalVelocity(const VelocityPose& velocity_goal)
        {
            _measurement = velocity_goal;
        }

        EdgeAccelerationHolonomic::EdgeAccelerationHolonomic()
        {
            resize(5);
        }

        void EdgeAccelerationHolonomic::computeError()
        {
            const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
            const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
            const VertexPose* pose3 = static_cast<const VertexPose*>(_vertices[2]);
            const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[3]);
            const VertexTimeDiff* dt2 = static_cast<const VertexTimeDiff*>(_vertices[4]);

            // Velocity & Acceleration
            geometry::Point diff1 = pose2->position() - pose1->position();
            geometry::Point diff2 = pose3->position() - pose2->position();

            double cos_theta1 = units::cos(pose1->theta());
            double sin_theta1 = units::sin(pose1->theta());
            double cos_theta2 = units::cos(pose2->theta());
            double sin_theta2 = units::sin(pose2->theta());

            // Transform pose2 into robot frame pose1
            units::Distance p1_dx = cos_theta1 * diff1.x() + sin_theta1 * diff1.y();
            units::Distance p1_dy = -sin_theta1 * diff1.x() + cos_theta1 * diff1.y();

            // Transform pose3 into robot frame pose2
            units::Distance p2_dx = cos_theta2 * diff2.x() + sin_theta2 * diff2.y();
            units::Distance p2_dy = -sin_theta2 * diff2.x() + cos_theta2 * diff2.y();

            units::Velocity vel1_x = p1_dx / dt1->dt();
            units::Velocity vel1_y = p1_dy / dt1->dt();
            units::Velocity vel2_x = p2_dx / dt2->dt();
            units::Velocity vel2_y = p2_dy / dt2->dt();

            units::Time dt12 = dt1->dt() + dt2->dt();

            units::Acceleration acc_x = (vel2_x - vel1_x) * 2.0 / dt12;
            units::Acceleration acc_y = (vel2_y - vel1_y) * 2.0 / dt12;

            _error[0] = penaltyBoundToInterval(acc_x, m_config->robot.acceleration_limit_x, m_config->optimization.penalty_epsilon * units::in / units::s / units::s)();
            _error[1] = penaltyBoundToInterval(acc_y, m_config->robot.acceleration_limit_y, m_config->optimization.penalty_epsilon * units::in / units::s / units::s)();

            // Angular Acceleration
            units::AngularVelocity omega1 = g2o::normalize_theta(pose2->theta().to(units::rad) - pose1->theta().to(units::rad)) * units::rad / dt1->dt();
            units::AngularVelocity omega2 = g2o::normalize_theta(pose3->theta().to(units::rad) - pose2->theta().to(units::rad)) * units::rad / dt2->dt();
            units::AngularAcceleration acc_rot = (omega2 - omega1) * 2.0 / dt12;

            _error[2] = penaltyBoundToInterval(acc_rot, m_config->robot.acceleration_limit_theta, m_config->optimization.penalty_epsilon * units::rad / units::s / units::s)();
        }

        void EdgeAccelerationHolonomicStart::setStartVelocity(const VelocityPose& velocity)
        {
            _measurement = velocity;
        }


        EdgeAccelerationHolonomicStart::EdgeAccelerationHolonomicStart()
        {
            resize(3);
        }

        void EdgeAccelerationHolonomicStart::computeError()
        {
            const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
            const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
            const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);

            // Velocity & Acceleration
            geometry::Point diff = pose2->position() - pose1->position();

            double cos_theta1 = units::cos(pose1->theta());
            double sin_theta1 = units::sin(pose1->theta());

            // Transform pose2 into robot frame pose1
            units::Distance p1_dx = cos_theta1 * diff.x() + sin_theta1 * diff.y();
            units::Distance p1_dy = -sin_theta1 * diff.x() + cos_theta1 * diff.y();

            units::Velocity vel1_x = _measurement.x();
            units::Velocity vel1_y = _measurement.y();
            units::Velocity vel2_x = p1_dx / dt->dt();
            units::Velocity vel2_y = p1_dy / dt->dt();

            units::Acceleration acc_x = (vel2_x - vel1_x) * 2.0 / dt->dt();
            units::Acceleration acc_y = (vel2_y - vel1_y) * 2.0 / dt->dt();

            _error[0] = penaltyBoundToInterval(acc_x, m_config->robot.acceleration_limit_x, m_config->optimization.penalty_epsilon * units::in / units::s / units::s)();
            _error[1] = penaltyBoundToInterval(acc_y, m_config->robot.acceleration_limit_y, m_config->optimization.penalty_epsilon * units::in / units::s / units::s)();

            // Angular Acceleration
            units::AngularVelocity omega1 = _measurement.omega();
            units::AngularVelocity omega2 = g2o::normalize_theta(pose2->theta().to(units::rad) - pose1->theta().to(units::rad)) * units::rad / dt->dt();
            units::AngularAcceleration acc_rot = (omega2 - omega1) * 2.0 / dt->dt();

            _error[2] = penaltyBoundToInterval(acc_rot, m_config->robot.acceleration_limit_theta, m_config->optimization.penalty_epsilon * units::rad / units::s / units::s)();
        }

        EdgeAccelerationHolonomicGoal::EdgeAccelerationHolonomicGoal()
        {
            resize(3);
        }

        void EdgeAccelerationHolonomicGoal::setGoalVelocity(const VelocityPose& velocity)
        {
            _measurement = velocity;
        }

        void EdgeAccelerationHolonomicGoal::computeError()
        {
            const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
            const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
            const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);

            // Velocity & Acceleration
            geometry::Point diff = pose2->position() - pose1->position();

            double cos_theta1 = units::cos(pose1->theta());
            double sin_theta1 = units::sin(pose1->theta());


            // Transform pose2 into robot frame pose1
            units::Distance p1_dx = cos_theta1 * diff.x() + sin_theta1 * diff.y();
            units::Distance p1_dy = -sin_theta1 * diff.x() + cos_theta1 * diff.y();

            units::Velocity vel1_x = p1_dx / dt->dt();
            units::Velocity vel1_y = p1_dy / dt->dt();
            units::Velocity vel2_x = _measurement.x();
            units::Velocity vel2_y = _measurement.y();

            units::Acceleration acc_x = (vel2_x - vel1_x) * 2.0 / dt->dt();
            units::Acceleration acc_y = (vel2_y - vel1_y) * 2.0 / dt->dt();

            _error[0] = penaltyBoundToInterval(acc_x, m_config->robot.acceleration_limit_x, m_config->optimization.penalty_epsilon * units::in / units::s / units::s)();
            _error[1] = penaltyBoundToInterval(acc_y, m_config->robot.acceleration_limit_y, m_config->optimization.penalty_epsilon * units::in / units::s / units::s)();

            // Angular Acceleration
            units::AngularVelocity omega1 = g2o::normalize_theta(pose2->theta().to(units::rad) - pose1->theta().to(units::rad)) * units::rad / dt->dt();
            units::AngularVelocity omega2 = _measurement.omega();
            units::AngularAcceleration acc_rot = (omega2 - omega1) * 2.0 / dt->dt();

            _error[2] = penaltyBoundToInterval(acc_rot, m_config->robot.acceleration_limit_theta, m_config->optimization.penalty_epsilon * units::rad / units::s / units::s)();
        }
    }
}
