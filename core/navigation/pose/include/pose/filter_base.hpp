#ifndef FILTER_BASE_HPP
#define FILTER_BASE_HPP

#include <vector>
#include <eigen3/Eigen/Core>

#include "pose/measurement.hpp"
#include "misc/logger.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pose
        {
            class FilterBase
            {
            public:
                /**
                 * Constructor
                 */
                FilterBase();

                /**
                 * Destructor
                 */
                virtual ~FilterBase();

                /**
                 * Resets filter to its unintialized state
                 */
                void reset();

                /**
                 * Computes a dynamic process noise covariance matrix using the parameterized state
                 *
                 * This allows us to, e.g., not increase the pose covariance values when the vehicle is not moving
                 *
                 * @param state - the STATE_SIZE state vector that is used to generate the dynamic process noise covariance
                 */
                void computeDynamicProcessNoiseCovariance(const Eigen::VectorXd& state, const double delta);

                /**
                 * Carries out the correction step in the predict/update cycle. This method must
                 * be implemented by subclasses.
                 *
                 * @param measurement - The measurement to fuse with the state estimate
                 */
                virtual void correct(const Measurement& measurement) = 0;

                /**
                 * Returns the control vector currently being used
                 */
                const Eigen::VectorXd& getControl();

                /**
                 * Returns the time at which the control term was issued
                 */
                double getControlTime();

                /**
                 * Returns the estimate error covariance
                 */
                const Eigen::MatrixXd& getEstimateErrorCovariance();

                /**
                 * Returns the filter's initialized status
                 */
                bool getInitializedStatus();

                /**
                 * Returns the most recent measurement time
                 */
                double getLastMeasurementTime();

                /**
                 * Returns the fitler's predicted state, i.e., the
                 * state estimate before correct() is called.
                 */
                const Eigen::VectorXd& getPredictedState();

                /**
                 * Returns the filter's process noise covariance
                 */
                const Eigen::MatrixXd& getProcessNoiseCovariance();

                /**
                 * Returns the sensor timeout (in seconds)
                 */
                double getSensorTimeout();

                /**
                 * Returns the filter state
                 */
                const Eigen::VectorXd& getState();

                /**
                 * Carries out the predict step in the predict/update cycle.
                 * Projects the state and error matrices forward using a model
                 * of the vehicle's motion. This method must be implemented by
                 * subclasses.
                 *
                 * @param reference_time - The time at which the prediction is being made
                 * @param delta - The time step over which to predict
                 */
                virtual void predict(const double reference_time, const double delta) = 0;

                /**
                 * Does some final preprocessing, carries out the predict/update cycle
                 *
                 * @param measurement - The measurement object to fuse into the filter
                 */
                virtual void processMeasurement(const Measurement& measurement);

                /**
                 * Sets the most recent control vector
                 *
                 * @param control - The control term to be applied
                 * @param control_time - The time at which the control in question was received
                 */
                void setControl(const Eigen::VectorXd& control, const double control_time);

                /**
                 * Sets the econtrol update vector and acceleration limits
                 *
                 * @param update - The values the control term affects
                 * @param control_timeout - Timeout value, in seconds, after which a control is considered stale
                 * @param acceleration_limits - The acceleration limits for the control variables
                 * @param acceleration_gains - Gains applied to the control term-derived acceleration
                 * @param deceleration_limits - The deceleration limits for the control variables
                 * @param deceleration_gains - Gains applied to the control term-derived deceleration
                 */
                void setControlParams(const std::vector<int>& update, const double control_timeout, const std::vector<double>& acceleration_limits, const std::vector<double>& acceleration_gains, const std::vector<double>& deceleration_limits, const std::vector<double>& deceleration_gains);

                /**
                 * Enables dynamic process noise covariance calculation
                 *
                 * @param dynamicProcessNoiseCovariance - Whether or not to compute dynamic process noise
                 *      covariance matrices
                 */
                void setUseDynamicProcessNoiseCovariance(const bool dynamicProcessNoiseCovariance);

                /**
                 * Manually sets the filter's estimate error covariance
                 *
                 * @param estimateErrorCovariance - The state to set as the filter's current state
                 */
                void setEstimateErrorCovariance(const Eigen::MatrixXd& estimateErrorCovariance);

                /**
                 * Sets the filter's last measurement time.
                 *
                 * @param lastMeasurementTime - The last measurement time of the filter
                 */
                void setLastMeasurementTime(const double lastMeasurementTime);

                /**
                 * Sets the process noise covariance for the filter.
                 *
                 * @param processNoiseCovariance - The STATE_SIZExSTATE_SIZE process noise covariance matrix
                 *      to use for the filter
                 */
                void setProcessNoiseCovariance(const Eigen::MatrixXd& processNoiseCovariance);

                /**
                 * Sets the sensor timeout
                 *
                 * @param sensorTimeout - The time, in seconds, for a sensor to be considered
                 *      having timed out
                 */
                void setSensorTimeout(const double sensorTimeout);

                /**
                 * Manually sets the filter's state
                 *
                 * @param state - The state to set as the filter's current state
                 */
                void setState(const Eigen::VectorXd& state);

                /**
                 * Ensures a given time delta is valid
                 *
                 * @param delta - The time delta, in seconds, to validate
                 */
                void validateDelta(double& delta);

            protected:
                inline double computedControlAcceleration(const double state, const double control, const double acceleration_limit, const double acceleration_gain, const double deceleration_limit, const double deceleration_gain)
                {
                    /*
                    misc::Logger::getInstance()->debug("---------- FilterBase::computeControlAcceleration ----------");
                     */

                    const double error = control - state;
                    const bool same_sign = std::fabs(error) <= std::fabs(control) + 0.01;
                    const double setpoint = same_sign ? control : 0.0;
                    const bool decelerating = std::fabs(setpoint) < std::fabs(state);
                    double limit = acceleration_limit;
                    double gain = acceleration_gain;

                    if(decelerating)
                    {
                        limit = deceleration_limit;
                        gain = deceleration_gain;
                    }

                    const double final_accel = std::min(std::max(gain * error, -limit), limit);

                    /*
                    misc::Logger::getInstance()->debug(fmt::format("Control value: {}", control));
                    misc::Logger::getInstance()->debug(fmt::format("State value: {}", state));
                    misc::Logger::getInstance()->debug(fmt::format("Error: {}", error));
                    misc::Logger::getInstance()->debug(fmt::format("Same sign: {}", same_sign ? "true" : "false"));
                    misc::Logger::getInstance()->debug(fmt::format("Setpoint: {}", setpoint));
                    misc::Logger::getInstance()->debug(fmt::format("Decelerating: {}", decelerating ? "true" : "false"));
                    misc::Logger::getInstance()->debug(fmt::format("Limit: {}", limit));
                    misc::Logger::getInstance()->debug(fmt::format("Gain: {}", gain));
                    misc::Logger::getInstance()->debug(fmt::format("Final: {}", final_accel));
                     */

                    return final_accel;
                }

                /**
                 * Keeps the state Euler angles in the range [-pi, pi]
                 */
                virtual void wrapStateAngles();

                /**
                 * Teset if innocation is within N-sigmas of covariance
                 *
                 * @param innovation - The difference between the measurement and the state
                 * @param inv_covariance - The innovation error
                 * @param n_sigmas - Number of standard deviations that are considered acceptable
                 * @return
                 */
                virtual bool checkMahalanobisThreshold(const Eigen::VectorXd& innovation, const Eigen::MatrixXd& inv_covariance, const double n_sigmas);

                /**
                 * Converts the control term to an acceleration to be applied in the prediction step
                 *
                 * @param reference_time - The time of the update
                 * @param prediction_delta - The amount of time over which we are carrying out our prediction
                 */
                void prepareControl(const double reference_time, const double prediction_delta);

                std::vector<double> m_acceleration_gains; //!< Gains applied to acceleration derived from the control term
                std::vector<double> m_acceleration_limits; //!< Caps the acceleration we apply from control input
                Eigen::VectorXd m_control_acceleration; //!< Gets updated every time we process a measurement and we have a valid control
                std::vector<double> m_deceleration_gains; //!< Gains applied to deceleration derived from the control term
                std::vector<double> m_deceleration_limits; //!< Caps the deceleration we apply from control input
                Eigen::VectorXd m_latest_control; //!< Latest control term
                std::vector<int> m_control_update_vector; //!< Which control variables are being used
                double m_control_timeout; //!< Timeout value, in seconds, after which a control is considered stale
                Eigen::MatrixXd m_covariance_epsilon; //!< Covariance matrices can be incredibly unstable. A small value is added at each iteration to help maintain its positive-definite property.
                Eigen::MatrixXd m_dynamic_process_noise_covariance; //!< Gets updated
                Eigen::MatrixXd m_estimate_error_covariance; //!< This matrix stores the total error in our position estimate
                Eigen::MatrixXd m_identity; //!< We need the identity for a few operations. Better to store it.
                bool m_initialized; //!< Whether or not we've received any measurements
                double m_last_measurement_time; //!< Tracks the time the filter was last updated using a measurement.
                double m_last_control_time; //!< The time of reception of the most recent control term
                Eigen::VectorXd m_predicted_state; //!< Holds the last predicted state of the filter
                Eigen::MatrixXd m_process_noise_covariance; //!<
                double m_sensor_timeout; //!< If we get a gap in measurements for some reason, we want the filter to continue estimating.
                Eigen::VectorXd m_state; //!< This is the robot's state vector
                Eigen::MatrixXd m_transfer_function; //!< The Kalman filter transfer function
                Eigen::MatrixXd m_transfer_function_jacobian; //!< The Kalman filter transfer function Jacobian

                bool m_use_control; //!< Whether or not we apply the control term
                bool m_use_dynamic_process_noise_covariance;
            };
        }
    }
}

#endif //FILTER_BASE_HPP
