#include "pose/ekf.hpp"
#include "pose/filter_common.hpp"
#include <misc/logger.hpp>

namespace rip
{
    namespace navigation
    {
        namespace pose
        {
            Ekf::Ekf(std::vector<double> args)
                : FilterBase()
            {
            }

            Ekf::~Ekf()
            {
            }

            void Ekf::correct(const Measurement& measurement)
            {
                // We don't want to update everything, so we need to build matrices that only update
                // the measured parts of our state vector. Throughout prediction and correction, we
                // attempt to maximize efficiency in Eigen.

                // First, determine how many state vector values we're updating
                std::vector<size_t> update_indices;
                for (size_t i = 0; i < measurement.updateVector().size(); ++i)
                {
                    if (measurement.updateVector()[i])
                    {
                        // Handle nan and inf values in measurements
                        if (std::isnan(measurement.measurement()(i)))
                        {
                            misc::Logger::getInstance()->debug("Value at index {} was nan. Excluding from update.\n", i);
                        }
                        else if (std::isinf(measurement.measurement()(i)))
                        {
                            misc::Logger::getInstance()->debug("Value at index {} was inf. Excluding from update.\n", i);
                        }
                        else
                        {
                            update_indices.push_back(i);
                        }
                    }
                }

                size_t update_size = update_indices.size();

                // Now set up the relevant matrices
                Eigen::VectorXd state_subset(update_size);                                  // x (in most literature)
                Eigen::VectorXd measurement_subset(update_size);                            // z
                Eigen::MatrixXd measurement_covariance_subset(update_size, update_size);    // R
                Eigen::MatrixXd state_to_measurement_subset(update_size, m_state.rows());   // H
                Eigen::MatrixXd kalman_gain_subset(m_state.rows(), update_size);            // K
                Eigen::VectorXd innovation_subset(update_size);                             // z - Hx

                state_subset.setZero();
                measurement_subset.setZero();
                measurement_covariance_subset.setZero();
                state_to_measurement_subset.setZero();
                kalman_gain_subset.setZero();
                innovation_subset.setZero();

                // Now build the sub-matrices from the full-sized matrices
                for (size_t i = 0; i < update_size; ++i)
                {
                    measurement_subset(i) = measurement.measurement()(update_indices[i]);
                    state_subset(i) = m_state(update_indices[i]);

                    for (size_t j = 0; j < update_size; ++j)
                    {
                        measurement_covariance_subset(i, j) = measurement.covariance()(update_indices[i], update_indices[j]);
                    }

                    // Handle negative (read: bad) covariances in the measurement. Rather
                    // than exclude the measurement or make up a covariance, just take
                    // the absolute value.
                    if (measurement_covariance_subset(i, i) < 0)
                    {
                        misc::Logger::getInstance()->debug("WARNING: Negative covariance for index {} of measurement (value is {}). Using absolute value...\n", i, measurement_covariance_subset(i, i));

                        measurement_covariance_subset(i, i) = std::fabs(measurement_covariance_subset(i, i));
                    }

                    // If the measurement variance for a given variable is very
                    // near 0 (as in e-50 or so) and the variance for that
                    // variable in the covariance matrix is also near zero, then
                    // the Kalman gain computation will blow up. Really, no
                    // measurement can be completely without error, so add a small
                    // amount in that case.
                    if (measurement_covariance_subset(i, i) < 1e-9)
                    {
                        misc::Logger::getInstance()->debug("WARNING: measurement had very small error covariance for index {}. Adding some noise to maintain filter stability.\n", update_indices[i]);

                        measurement_covariance_subset(i, i) = 1e-9;
                    }
                }

                // The state-to-measurement function, h, will now be a measurement_size x full_state_size
                // matrix, with ones in the (i, i) locations of the values to be updated
                for (size_t i = 0; i < update_size; ++i)
                {
                    state_to_measurement_subset(i, update_indices[i]) = 1;
                }

                // (1) Compute the Kalman gain: K = (PH') / (HPH' + R)
                Eigen::MatrixXd pht = m_estimate_error_covariance * state_to_measurement_subset.transpose();
                Eigen::MatrixXd hphr_inv  = (state_to_measurement_subset * pht + measurement_covariance_subset).inverse();
                kalman_gain_subset.noalias() = pht * hphr_inv;

                innovation_subset = (measurement_subset - state_subset);

                // Wrap angles in the innovation
                for (size_t i = 0; i < update_size; ++i)
                {
                    if (update_indices[i] == static_cast<unsigned long>(StateMembers::kRoll)  ||
                        update_indices[i] == static_cast<unsigned long>(StateMembers::kPitch) ||
                        update_indices[i] == static_cast<unsigned long>(StateMembers::kYaw))
                    {
                        while (innovation_subset(i) < -M_PI)
                        {
                            innovation_subset(i) += M_2_PI;
                        }

                        while (innovation_subset(i) > M_PI)
                        {
                            innovation_subset(i) -= M_2_PI;
                        }
                    }
                }

                // (2) Check Mahalanobis distance between mapped measurement and state.
                if (checkMahalanobisThreshold(innovation_subset, hphr_inv, measurement.mahalanobisThreshold()))
                {
                    // (3) Apply the gain to the difference between the state and measurement: x = x + K(z - Hx)
                    m_state.noalias() += kalman_gain_subset * innovation_subset;

                    // (4) Update the estimate error covariance using the Joseph form: (I - KH)P(I - KH)' + KRK'
                    Eigen::MatrixXd gain_residual = m_identity;
                    gain_residual.noalias() -= kalman_gain_subset * state_to_measurement_subset;
                    m_estimate_error_covariance = gain_residual * m_estimate_error_covariance * gain_residual.transpose();
                    m_estimate_error_covariance.noalias() += kalman_gain_subset * measurement_covariance_subset * kalman_gain_subset.transpose();

                    // Handle wrapping of angles
                    wrapStateAngles();
                }
            }

            void Ekf::predict(const double reference_time, const double delta)
            {
                std::stringstream ss;
                ss << m_state;
                misc::Logger::getInstance()->debug("Ekf::predict\tdelta is {}\tstate is {}\n", delta, ss.str());

                double roll = m_state(static_cast<int>(StateMembers::kRoll));
                double pitch = m_state(static_cast<int>(StateMembers::kPitch));
                double yaw = m_state(static_cast<int>(StateMembers::kYaw));
                double xVel = m_state(static_cast<int>(StateMembers::kVx));
                double yVel = m_state(static_cast<int>(StateMembers::kVy));
                double zVel = m_state(static_cast<int>(StateMembers::kVz));
                double rollVel = m_state(static_cast<int>(StateMembers::kVroll));
                double pitchVel = m_state(static_cast<int>(StateMembers::kVpitch));
                double yawVel = m_state(static_cast<int>(StateMembers::kVyaw));
                double xAcc = m_state(static_cast<int>(StateMembers::kAx));
                double yAcc = m_state(static_cast<int>(StateMembers::kAy));
                double zAcc = m_state(static_cast<int>(StateMembers::kAz));

                // We'll need these trig calculations a lot.
                double sp = std::sin(pitch);
                double cp = std::cos(pitch);

                double sr = std::sin(roll);
                double cr = std::cos(roll);

                double sy = std::sin(yaw);
                double cy = std::cos(yaw);

                prepareControl(reference_time, delta);

                // Prepare the transfer function
                m_transfer_function(static_cast<int>(StateMembers::kX), static_cast<int>(StateMembers::kVx)) = cy * cp * delta;
                m_transfer_function(static_cast<int>(StateMembers::kX), static_cast<int>(StateMembers::kVy)) = (cy * sp * sr - sy * cr) * delta;
                m_transfer_function(static_cast<int>(StateMembers::kX), static_cast<int>(StateMembers::kVz)) = (cy * sp * cr + sy * sr) * delta;
                m_transfer_function(static_cast<int>(StateMembers::kX), static_cast<int>(StateMembers::kAx)) = 0.5 * m_transfer_function(static_cast<int>(StateMembers::kX), static_cast<int>(StateMembers::kVx)) * delta;
                m_transfer_function(static_cast<int>(StateMembers::kX), static_cast<int>(StateMembers::kAy)) = 0.5 * m_transfer_function(static_cast<int>(StateMembers::kX), static_cast<int>(StateMembers::kVy)) * delta;
                m_transfer_function(static_cast<int>(StateMembers::kX), static_cast<int>(StateMembers::kAz)) = 0.5 * m_transfer_function(static_cast<int>(StateMembers::kX), static_cast<int>(StateMembers::kVz)) * delta;
                m_transfer_function(static_cast<int>(StateMembers::kY), static_cast<int>(StateMembers::kVx)) = sy * cp * delta;
                m_transfer_function(static_cast<int>(StateMembers::kY), static_cast<int>(StateMembers::kVy)) = (sy * sp * sr + cy * cr) * delta;
                m_transfer_function(static_cast<int>(StateMembers::kY), static_cast<int>(StateMembers::kVz)) = (sy * sp * cr - cy * sr) * delta;
                m_transfer_function(static_cast<int>(StateMembers::kY), static_cast<int>(StateMembers::kAx)) = 0.5 * m_transfer_function(static_cast<int>(StateMembers::kY), static_cast<int>(StateMembers::kVx)) * delta;
                m_transfer_function(static_cast<int>(StateMembers::kY), static_cast<int>(StateMembers::kAy)) = 0.5 * m_transfer_function(static_cast<int>(StateMembers::kY), static_cast<int>(StateMembers::kVy)) * delta;
                m_transfer_function(static_cast<int>(StateMembers::kY), static_cast<int>(StateMembers::kAz)) = 0.5 * m_transfer_function(static_cast<int>(StateMembers::kY), static_cast<int>(StateMembers::kVz)) * delta;
                m_transfer_function(static_cast<int>(StateMembers::kZ), static_cast<int>(StateMembers::kVx)) = -sp * delta;
                m_transfer_function(static_cast<int>(StateMembers::kZ), static_cast<int>(StateMembers::kVy)) = cp * sr * delta;
                m_transfer_function(static_cast<int>(StateMembers::kZ), static_cast<int>(StateMembers::kVz)) = cp * cr * delta;
                m_transfer_function(static_cast<int>(StateMembers::kZ), static_cast<int>(StateMembers::kAx)) = 0.5 * m_transfer_function(static_cast<int>(StateMembers::kZ), static_cast<int>(StateMembers::kVx)) * delta;
                m_transfer_function(static_cast<int>(StateMembers::kZ), static_cast<int>(StateMembers::kAy)) = 0.5 * m_transfer_function(static_cast<int>(StateMembers::kZ), static_cast<int>(StateMembers::kVy)) * delta;
                m_transfer_function(static_cast<int>(StateMembers::kZ), static_cast<int>(StateMembers::kAz)) = 0.5 * m_transfer_function(static_cast<int>(StateMembers::kZ), static_cast<int>(StateMembers::kVz)) * delta;
                m_transfer_function(static_cast<int>(StateMembers::kRoll), static_cast<int>(StateMembers::kVroll)) = m_transfer_function(static_cast<int>(StateMembers::kX), static_cast<int>(StateMembers::kVx));
                m_transfer_function(static_cast<int>(StateMembers::kRoll), static_cast<int>(StateMembers::kVpitch)) = m_transfer_function(static_cast<int>(StateMembers::kX), static_cast<int>(StateMembers::kVy));
                m_transfer_function(static_cast<int>(StateMembers::kRoll), static_cast<int>(StateMembers::kVyaw)) = m_transfer_function(static_cast<int>(StateMembers::kX), static_cast<int>(StateMembers::kVz));
                m_transfer_function(static_cast<int>(StateMembers::kPitch), static_cast<int>(StateMembers::kVroll)) = m_transfer_function(static_cast<int>(StateMembers::kY), static_cast<int>(StateMembers::kVx));
                m_transfer_function(static_cast<int>(StateMembers::kPitch), static_cast<int>(StateMembers::kVpitch)) = m_transfer_function(static_cast<int>(StateMembers::kY), static_cast<int>(StateMembers::kVy));
                m_transfer_function(static_cast<int>(StateMembers::kPitch), static_cast<int>(StateMembers::kVyaw)) = m_transfer_function(static_cast<int>(StateMembers::kY), static_cast<int>(StateMembers::kVz));
                m_transfer_function(static_cast<int>(StateMembers::kYaw), static_cast<int>(StateMembers::kVroll)) = m_transfer_function(static_cast<int>(StateMembers::kZ), static_cast<int>(StateMembers::kVx));
                m_transfer_function(static_cast<int>(StateMembers::kYaw), static_cast<int>(StateMembers::kVpitch)) = m_transfer_function(static_cast<int>(StateMembers::kZ), static_cast<int>(StateMembers::kVy));
                m_transfer_function(static_cast<int>(StateMembers::kYaw), static_cast<int>(StateMembers::kVyaw)) = m_transfer_function(static_cast<int>(StateMembers::kZ), static_cast<int>(StateMembers::kVz));
                m_transfer_function(static_cast<int>(StateMembers::kVx), static_cast<int>(StateMembers::kAx)) = delta;
                m_transfer_function(static_cast<int>(StateMembers::kVy), static_cast<int>(StateMembers::kAy)) = delta;
                m_transfer_function(static_cast<int>(StateMembers::kVz), static_cast<int>(StateMembers::kAz)) = delta;

                // Prepare the transfer function Jacobian. This function is analytically derived from the
                // transfer function.
                double xCoeff = 0.0;
                double yCoeff = 0.0;
                double zCoeff = 0.0;
                double oneHalfATSquared = 0.5 * delta * delta;

                yCoeff = cy * sp * cr + sy * sr;
                zCoeff = -cy * sp * sr + sy * cr;
                double dFx_dR = (yCoeff * yVel + zCoeff * zVel) * delta +
                                (yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
                double dFR_dR = 1 + (yCoeff * pitchVel + zCoeff * yawVel) * delta;

                xCoeff = -cy * sp;
                yCoeff = cy * cp * sr;
                zCoeff = cy * cp * cr;
                double dFx_dP = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
                                (xCoeff * xAcc + yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
                double dFR_dP = (xCoeff * rollVel + yCoeff * pitchVel + zCoeff * yawVel) * delta;

                xCoeff = -sy * cp;
                yCoeff = -sy * sp * sr - cy * cr;
                zCoeff = -sy * sp * cr + cy * sr;
                double dFx_dY = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
                                (xCoeff * xAcc + yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
                double dFR_dY = (xCoeff * rollVel + yCoeff * pitchVel + zCoeff * yawVel) * delta;

                yCoeff = sy * sp * cr - cy * sr;
                zCoeff = -sy * sp * sr - cy * cr;
                double dFy_dR = (yCoeff * yVel + zCoeff * zVel) * delta +
                                (yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
                double dFP_dR = (yCoeff * pitchVel + zCoeff * yawVel) * delta;

                xCoeff = -sy * sp;
                yCoeff = sy * cp * sr;
                zCoeff = sy * cp * cr;
                double dFy_dP = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
                                (xCoeff * xAcc + yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
                double dFP_dP = 1 + (xCoeff * rollVel + yCoeff * pitchVel + zCoeff * yawVel) * delta;

                xCoeff = cy * cp;
                yCoeff = cy * sp * sr - sy * cr;
                zCoeff = cy * sp * cr + sy * sr;
                double dFy_dY = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
                                (xCoeff * xAcc + yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
                double dFP_dY = (xCoeff * rollVel + yCoeff * pitchVel + zCoeff * yawVel) * delta;

                yCoeff = cp * cr;
                zCoeff = -cp * sr;
                double dFz_dR = (yCoeff * yVel + zCoeff * zVel) * delta +
                                (yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
                double dFY_dR = (yCoeff * pitchVel + zCoeff * yawVel) * delta;

                xCoeff = -cp;
                yCoeff = -sp * sr;
                zCoeff = -sp * cr;
                double dFz_dP = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
                                (xCoeff * xAcc + yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
                double dFY_dP = (xCoeff * rollVel + yCoeff * pitchVel + zCoeff * yawVel) * delta;

                // Much of the transfer function Jacobian is identical to the transfer function
                m_transfer_function_jacobian = m_transfer_function;
                m_transfer_function_jacobian(static_cast<int>(StateMembers::kX), static_cast<int>(StateMembers::kRoll)) = dFx_dR;
                m_transfer_function_jacobian(static_cast<int>(StateMembers::kX), static_cast<int>(StateMembers::kPitch)) = dFx_dP;
                m_transfer_function_jacobian(static_cast<int>(StateMembers::kX), static_cast<int>(StateMembers::kYaw)) = dFx_dY;
                m_transfer_function_jacobian(static_cast<int>(StateMembers::kY), static_cast<int>(StateMembers::kRoll)) = dFy_dR;
                m_transfer_function_jacobian(static_cast<int>(StateMembers::kY), static_cast<int>(StateMembers::kPitch)) = dFy_dP;
                m_transfer_function_jacobian(static_cast<int>(StateMembers::kY), static_cast<int>(StateMembers::kYaw)) = dFy_dY;
                m_transfer_function_jacobian(static_cast<int>(StateMembers::kZ), static_cast<int>(StateMembers::kRoll)) = dFz_dR;
                m_transfer_function_jacobian(static_cast<int>(StateMembers::kZ), static_cast<int>(StateMembers::kPitch)) = dFz_dP;
                m_transfer_function_jacobian(static_cast<int>(StateMembers::kRoll), static_cast<int>(StateMembers::kRoll)) = dFR_dR;
                m_transfer_function_jacobian(static_cast<int>(StateMembers::kRoll), static_cast<int>(StateMembers::kPitch)) = dFR_dP;
                m_transfer_function_jacobian(static_cast<int>(StateMembers::kRoll), static_cast<int>(StateMembers::kYaw)) = dFR_dY;
                m_transfer_function_jacobian(static_cast<int>(StateMembers::kPitch), static_cast<int>(StateMembers::kRoll)) = dFP_dR;
                m_transfer_function_jacobian(static_cast<int>(StateMembers::kPitch), static_cast<int>(StateMembers::kPitch)) = dFP_dP;
                m_transfer_function_jacobian(static_cast<int>(StateMembers::kPitch), static_cast<int>(StateMembers::kYaw)) = dFP_dY;
                m_transfer_function_jacobian(static_cast<int>(StateMembers::kYaw), static_cast<int>(StateMembers::kRoll)) = dFY_dR;
                m_transfer_function_jacobian(static_cast<int>(StateMembers::kYaw), static_cast<int>(StateMembers::kPitch)) = dFY_dP;

                Eigen::MatrixXd *process_noise_covariance = &m_process_noise_covariance;

                if (m_use_dynamic_process_noise_covariance)
                {
                    computeDynamicProcessNoiseCovariance(m_state, delta);
                    process_noise_covariance = &m_dynamic_process_noise_covariance;
                }

                // (1) Apply control terms, which are actually accelerations
                m_state(static_cast<int>(StateMembers::kVroll)) += m_control_acceleration(static_cast<int>(ControlMembers::kVroll)) * delta;
                m_state(static_cast<int>(StateMembers::kVpitch)) += m_control_acceleration(static_cast<int>(ControlMembers::kVpitch)) * delta;
                m_state(static_cast<int>(StateMembers::kVyaw)) += m_control_acceleration(static_cast<int>(ControlMembers::kVyaw)) * delta;

                m_state(static_cast<int>(StateMembers::kAx)) = (m_control_update_vector[static_cast<int>(ControlMembers::kVx)] ?
                                              m_control_acceleration(static_cast<int>(ControlMembers::kVx)) : m_state(static_cast<int>(StateMembers::kAx)));
                m_state(static_cast<int>(StateMembers::kAy)) = (m_control_update_vector[static_cast<int>(ControlMembers::kVy)] ?
                                              m_control_acceleration(static_cast<int>(ControlMembers::kVy)) : m_state(static_cast<int>(StateMembers::kAy)));
                m_state(static_cast<int>(StateMembers::kAz)) = (m_control_update_vector[static_cast<int>(ControlMembers::kVz)] ?
                                              m_control_acceleration(static_cast<int>(ControlMembers::kVz)) : m_state(static_cast<int>(StateMembers::kAz)));

                // (2) Project the state forward: x = Ax + Bu (really, x = f(x, u))
                m_state = m_transfer_function * m_state;

                // Handle wrapping
                wrapStateAngles();

                std::stringstream ss1, ss2;
                ss1 << m_state;
                ss2 << m_estimate_error_covariance;
                misc::Logger::getInstance()->debug("Predicted state is:\n{}\nCurrent estimate error covariance is:\n{}\n", ss1.str(), ss2.str());

                // (3) Project the error forward: P = J * P * J' + Q
                m_estimate_error_covariance = (m_transfer_function_jacobian * m_estimate_error_covariance * m_transfer_function_jacobian.transpose());
                m_estimate_error_covariance.noalias() += delta * (*process_noise_covariance);
            }
        }
    }
}