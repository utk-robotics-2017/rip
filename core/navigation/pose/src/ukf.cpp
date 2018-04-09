#include "pose/ukf.hpp"
#include "pose/filter_common.hpp"

#include <Eigen/Cholesky>

namespace rip
{
    namespace navigation
    {
        namespace pose
        {
            Ukf::Ukf(std::vector<double> args)
                : FilterBase()
                , m_uncorrected(true)
            {
                const double alpha = args[0];
                const double kappa = args[1];
                const double beta = args[2];

                size_t sigma_count = (kStateSize << 1) + 1;
                m_sigma_points.resize(sigma_count, Eigen::VectorXd(kStateSize));

                // Prepare constants
                m_lambda = alpha * alpha * (kStateSize + kappa) - kStateSize;

                m_state_weights.resize(sigma_count);
                m_covar_weights.resize(sigma_count);

                m_state_weights[0] = m_lambda / (kStateSize + m_lambda);
                m_covar_weights[0] = m_state_weights[0] + (1 - (alpha * alpha) + beta);
                m_sigma_points[0].setZero();
                for(size_t i = 1; i < sigma_count; ++i)
                {
                    m_sigma_points[i].setZero();
                    m_state_weights[i] = 1/ (2 * (kStateSize + m_lambda));
                    m_covar_weights[i] = m_state_weights[i];
                }
            }

            Ukf::~Ukf()
            {
            }

            void Ukf::correct(const Measurement& measurement)
            {
                // In our implementation, it may be that after we call predict once, we call correct
                // several times in succession (multiple measurements with different time stamps). In
                // that event, the sigma points need to be updated to reflect the current state.
                // Throughout prediction and correction, we attempt to maximize efficiency in Eigen.
                if(!m_uncorrected)
                {
                    // Take the square root of a small fraction of the m_estimate_error_covariance using LL' decomposition
                    m_weighted_covar_sqrt = ((kStateSize + m_lambda) * m_estimate_error_covariance).llt().matrixL();

                    // Compute sigma points

                    // First sigma point is the current state
                    m_sigma_points[0] = m_state;

                    // Next kStateSize sigma points are state + m_weighted_covar_sqrt[ith column]
                    // kStateSize sigma points after that are state - m_weighted_covar_sqrt[ith column]
                    for (size_t sigma_idx = 0; sigma_idx < kStateSize; ++sigma_idx)
                    {
                        m_sigma_points[sigma_idx + 1] = m_state + m_weighted_covar_sqrt.col(sigma_idx);
                        m_sigma_points[sigma_idx + 1 + kStateSize] = m_state - m_weighted_covar_sqrt.col(sigma_idx);
                    }
                }

                // We don't want to update everything, so we need to build matrices that only update
                // the measured parts of our state vector

                // First, determine how many state vector values we're updating
                std::vector<size_t> update_indices;
                for (size_t i = 0; i < measurement.updateVector().size(); ++i)
                {
                    if (measurement.updateVector()[i])
                    {
                        // Handle nan and inf values in measurements
                        if (std::isnan(measurement.measurement()(i)))
                        {
                            misc::Logger::getInstance()->debug("Value at index {} was nan. Excluding from update.", i);
                        }
                        else if (std::isinf(measurement.measurement()(i)))
                        {
                            misc::Logger::getInstance()->debug("Value at index {} was inf. Excluding from update.", i);
                        }
                        else
                        {
                            update_indices.push_back(i);
                        }
                    }
                }

                size_t updateSize = update_indices.size();

                // Now set up the relevant matrices
                Eigen::VectorXd stateSubset(updateSize);                              // x (in most literature)
                Eigen::VectorXd measurementSubset(updateSize);                        // z
                Eigen::MatrixXd measurementCovarianceSubset(updateSize, updateSize);  // R
                Eigen::MatrixXd stateToMeasurementSubset(updateSize, kStateSize);     // H
                Eigen::MatrixXd kalmanGainSubset(kStateSize, updateSize);             // K
                Eigen::VectorXd innovationSubset(updateSize);                         // z - Hx
                Eigen::VectorXd predictedMeasurement(updateSize);
                Eigen::VectorXd sigmaDiff(updateSize);
                Eigen::MatrixXd predictedMeasCovar(updateSize, updateSize);
                Eigen::MatrixXd crossCovar(kStateSize, updateSize);


                std::vector<Eigen::VectorXd> sigmaPointMeasurements(m_sigma_points.size(), Eigen::VectorXd(updateSize));

                stateSubset.setZero();
                measurementSubset.setZero();
                measurementCovarianceSubset.setZero();
                stateToMeasurementSubset.setZero();
                kalmanGainSubset.setZero();
                innovationSubset.setZero();
                predictedMeasurement.setZero();
                predictedMeasCovar.setZero();
                crossCovar.setZero();

                // Now build the sub-matrices from the full-sized matrices
                for (size_t i = 0; i < updateSize; ++i)
                {
                    measurementSubset(i) = measurement.measurement()(update_indices[i]);
                    stateSubset(i) = m_state(update_indices[i]);

                    for (size_t j = 0; j < updateSize; ++j)
                    {
                        measurementCovarianceSubset(i, j) = measurement.covariance()(update_indices[i], update_indices[j]);
                    }

                    // Handle negative (read: bad) covariances in the measurement. Rather
                    // than exclude the measurement or make up a covariance, just take
                    // the absolute value.
                    if (measurementCovarianceSubset(i, i) < 0)
                    {
                        std::stringstream ss;
                        ss << measurementCovarianceSubset(i,i);
                        misc::Logger::getInstance()->debug("WARNING: Negative covariance for index {} of measurement (value is {}). Using absolute value...", i, ss.str());

                        measurementCovarianceSubset(i, i) = ::fabs(measurementCovarianceSubset(i, i));
                    }

                    // If the measurement variance for a given variable is very
                    // near 0 (as in e-50 or so) and the variance for that
                    // variable in the covariance matrix is also near zero, then
                    // the Kalman gain computation will blow up. Really, no
                    // measurement can be completely without error, so add a small
                    // amount in that case.
                    if (measurementCovarianceSubset(i, i) < 1e-9)
                    {
                        measurementCovarianceSubset(i, i) = 1e-9;

                        misc::Logger::getInstance()->debug("WARNING: measurement had very small error covariance for index {}. Adding some noise to maintain filter stability.", update_indices[i]);
                    }
                }

                // The state-to-measurement function, h, will now be a measurement_size x full_kStateSize
                // matrix, with ones in the (i, i) locations of the values to be updated
                for (size_t i = 0; i < updateSize; ++i)
                {
                    stateToMeasurementSubset(i, update_indices[i]) = 1;
                }

                /*
                FB_DEBUG("Current state subset is:\n" << stateSubset <<
                                                      "\nMeasurement subset is:\n" << measurementSubset <<
                                                      "\nMeasurement covariance subset is:\n" << measurementCovarianceSubset <<
                                                      "\nState-to-measurement subset is:\n" << stateToMeasurementSubset << "\n");
                                                      */

                // (1) Generate sigma points, use them to generate a predicted measurement
                for (size_t sigmaInd = 0; sigmaInd < m_sigma_points.size(); ++sigmaInd)
                {
                    sigmaPointMeasurements[sigmaInd] = stateToMeasurementSubset * m_sigma_points[sigmaInd];
                    predictedMeasurement.noalias() += m_state_weights[sigmaInd] * sigmaPointMeasurements[sigmaInd];
                }

                // (2) Use the sigma point measurements and predicted measurement to compute a predicted
                // measurement covariance matrix P_zz and a state/measurement cross-covariance matrix P_xz.
                for (size_t sigmaInd = 0; sigmaInd < m_sigma_points.size(); ++sigmaInd)
                {
                    sigmaDiff = sigmaPointMeasurements[sigmaInd] - predictedMeasurement;
                    predictedMeasCovar.noalias() += m_covar_weights[sigmaInd] * (sigmaDiff * sigmaDiff.transpose());
                    crossCovar.noalias() += m_covar_weights[sigmaInd] * ((m_sigma_points[sigmaInd] - m_state) * sigmaDiff.transpose());
                }

                // (3) Compute the Kalman gain, making sure to use the actual measurement covariance: K = P_xz * (P_zz + R)^-1
                Eigen::MatrixXd invInnovCov = (predictedMeasCovar + measurementCovarianceSubset).inverse();
                kalmanGainSubset = crossCovar * invInnovCov;

                // (4) Apply the gain to the difference between the actual and predicted measurements: x = x + K(z - z_hat)
                innovationSubset = (measurementSubset - predictedMeasurement);

                // Wrap angles in the innovation
                for (size_t i = 0; i < updateSize; ++i)
                {
                    if (update_indices[i] == static_cast<unsigned long>(StateMembers::kRoll)  ||
                            update_indices[i] == static_cast<unsigned long>(StateMembers::kPitch) ||
                            update_indices[i] == static_cast<unsigned long>(StateMembers::kYaw))
                    {
                        while (innovationSubset(i) < -M_PI)
                        {
                            innovationSubset(i) += M_2_PI;
                        }

                        while (innovationSubset(i) > M_PI)
                        {
                            innovationSubset(i) -= M_2_PI;
                        }
                    }
                }

                // (5) Check Mahalanobis distance of innovation
                if (checkMahalanobisThreshold(innovationSubset, invInnovCov, measurement.mahalanobisThreshold()))
                {
                    m_state.noalias() += kalmanGainSubset * innovationSubset;

                    // (6) Compute the new estimate error covariance P = P - (K * P_zz * K')
                    m_estimate_error_covariance.noalias() -= (kalmanGainSubset * predictedMeasCovar * kalmanGainSubset.transpose());

                    wrapStateAngles();

                    // Mark that we need to re-compute sigma points for successive corrections
                    m_uncorrected = false;

                    std::stringstream ss;
                    ss << m_state;
                    misc::Logger::getInstance()->debug("Predicated: Corrected full state is:{}", ss.str());
                }
            }

            void Ukf::predict(const double referenceTime, const double delta)
            {
                std::stringstream ss;
                ss << m_state;
                misc::Logger::getInstance()->debug("Ukf::predict\tdelta is {}\tstate is {}", delta, ss.str());

                double roll = m_state(static_cast<int>(StateMembers::kRoll));
                double pitch = m_state(static_cast<int>(StateMembers::kPitch));
                double yaw = m_state(static_cast<int>(StateMembers::kYaw));

                // We'll need these trig calculations a lot.
                double sp = std::sin(pitch);
                double cp = std::cos(pitch);

                double sr = std::sin(roll);
                double cr = std::cos(roll);

                double sy = std::sin(yaw);
                double cy = std::cos(yaw);

                prepareControl(referenceTime, delta);

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

                // (1) Take the square root of a small fraction of the m_estimate_error_covariance using LL' decomposition
                m_weighted_covar_sqrt = ((kStateSize + m_lambda) * m_estimate_error_covariance).llt().matrixL();

                // (2) Compute sigma points *and* pass them through the transfer function to save
                // the extra loop

                // First sigma point is the current state
                m_sigma_points[0] = m_transfer_function * m_state;

                // Next kStateSize sigma points are state + m_weighted_covar_sqrt[ith column]
                // kStateSize sigma points after that are state - m_weighted_covar_sqrt[ith column]
                for (size_t sigmaInd = 0; sigmaInd < kStateSize; ++sigmaInd)
                {
                    m_sigma_points[sigmaInd + 1] = m_transfer_function * (m_state + m_weighted_covar_sqrt.col(sigmaInd));
                    m_sigma_points[sigmaInd + 1 + kStateSize] = m_transfer_function * (m_state - m_weighted_covar_sqrt.col(sigmaInd));
                }

                // (3) Sum the weighted sigma points to generate a new state prediction
                m_state.setZero();
                for (size_t sigmaInd = 0; sigmaInd < m_sigma_points.size(); ++sigmaInd)
                {
                    m_state.noalias() += m_state_weights[sigmaInd] * m_sigma_points[sigmaInd];
                }

                // (4) Now us the sigma points and the predicted state to compute a predicted covariance
                m_estimate_error_covariance.setZero();
                Eigen::VectorXd sigmaDiff(kStateSize);
                for (size_t sigmaInd = 0; sigmaInd < m_sigma_points.size(); ++sigmaInd)
                {
                    sigmaDiff = (m_sigma_points[sigmaInd] - m_state);
                    m_estimate_error_covariance.noalias() += m_covar_weights[sigmaInd] * (sigmaDiff * sigmaDiff.transpose());
                }

                // (5) Not strictly in the theoretical UKF formulation, but necessary here
                // to ensure that we actually incorporate the m_process_noise_covariance
                Eigen::MatrixXd *processNoiseCovariance = &m_process_noise_covariance;

                if (m_use_dynamic_process_noise_covariance)
                {
                    computeDynamicProcessNoiseCovariance(m_state, delta);
                    processNoiseCovariance = &m_dynamic_process_noise_covariance;
                }

                m_estimate_error_covariance.noalias() += delta * (*processNoiseCovariance);

                // Keep the angles bounded
                wrapStateAngles();

                // Mark that we can keep these sigma points
                m_uncorrected = true;

                ss.clear();
                ss << m_state;
                misc::Logger::getInstance()->debug("Predicted state is: {}", ss.str());
            }
        }
    }
}
