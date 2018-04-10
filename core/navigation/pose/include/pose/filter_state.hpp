#ifndef FILTER_STATE_HPP
#define FILTER_STATE_HPP

#include <eigen3/Eigen/Core>

namespace rip
{
    namespace navigation
    {
        namespace pose
        {
            /**
             * Structure for storing and comparing filter states
             *
             * This structure is useful when higher-level classes need to
             * remember filter history.
             *
             * Measurement units are in meters and radians.
             * Times are real-values and measured in seconds.
             */
            class FilterState
            {
            public:
                /**
                 * Constructor
                 */
                FilterState();

                /**
                 * Returns the filter state
                 */
                Eigen::VectorXd& state();

                /**
                 * Returns the filter state
                 */
                const Eigen::VectorXd& state() const;

                /**
                 * Returns the error covariance matrix
                 */
                Eigen::MatrixXd& errorCovariance();

                /**
                 * Returns the error covariance matrix
                 */
                const Eigen::MatrixXd& errorCovariance() const;

                /**
                 * Returns the latest control vector
                 */
                Eigen::VectorXd& latestControl();

                /**
                 * Returns the latest control vector
                 */
                const Eigen::VectorXd& latestControl() const;

                /**
                 * Returns the time stamp of the most recent measurement
                 */
                double& lastMeasurementTime();

                /**
                 * Returns the time stamp of the most recent measurement
                 */
                double lastMeasurementTime() const;

                /**
                 * Returns the time stamp of the most recent control
                 */
                double& lastControlTime();

                /**
                 * Returns the time stamp of the most recent control
                 */
                double lastControlTime() const;

                /**
                 * Used for sorting the queue from the latest to earliest time stamps.
                 */
                bool operator()(const FilterState& lhs, const FilterState& rhs);

            private:
                Eigen::VectorXd m_state; //!< The filter state vector
                Eigen::MatrixXd m_estimate_error_covariance; //!< The filter error covariance matrix
                Eigen::VectorXd m_latest_control; //!< The most recent control vector
                double m_last_measurement_time; //!< The time stamp of the most recent measurement for the filter
                double m_last_control_time; //!< The time stamp of the most recent control term

            };
        }
    }
}

#endif //FILTER_STATE_HPP
