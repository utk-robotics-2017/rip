#ifndef MEASUREMENT_HPP
#define MEASUREMENT_HPP

#include <eigen3/Eigen/Core>
#include <memory>
#include <vector>

namespace rip
{
    namespace navigation
    {
        namespace pose
        {
            /**
             * Structure used for storing and comparing measurements
             *
             * Measurement units are in meters and radians.
             * Times are real-values and measured in seconds.
             */
            class Measurement
            {
            public:
                Measurement();

                /**
                 * Returns the measurement
                 */
                Eigen::VectorXd& measurement();

                /**
                 * Returns the measurement
                 */
                const Eigen::VectorXd& measurement() const;

                /**
                 * Returns the covariance
                 */
                Eigen::MatrixXd& covariance();

                /**
                 * Returns the covariance
                 */
                const Eigen::MatrixXd& covariance() const;

                /**
                 * Returns a vector that defines the variables within
                 * this measurement actually get passed into the filter.
                 */
                std::vector<int>& updateVector();

                /**
                 * Returns a vector that defines the variables within
                 * this measurement actually get passed into the filter.
                 */
                const std::vector<int>& updateVector() const;

                /**
                 * Returns the timestamp of this measurement
                 */
                double& time();

                /**
                 * Returns the timestamp of this measurement
                 */
                double time() const;

                /**
                 * Returns the Mahalanobis distance threshold in number of sigmas
                 */
                double& mahalanobisThreshold();

                /**
                 * Returns the Mahalanobis distance threshold in number of sigmas
                 */
                double mahalanobisThreshold() const;

                /**
                 * Returns the most recent control vector
                 */
                Eigen::VectorXd& lastControl();

                /**
                 * Returns the most recent control vector
                 */
                const Eigen::VectorXd& lastControl() const;

                /**
                 * Returns the time stamp of the most recent control vector
                 */
                double& lastControlTime();

                /**
                 * Returns the time stamp of the most recent control vector
                 */
                double lastControlTime() const;

                /**
                 * Gives the earlier time the priority
                 */
                bool operator()(const std::shared_ptr<Measurement>& lhs, const std::shared_ptr<Measurement>& rhs);

                /**
                 * Gives the earlier time the priority
                 */
                bool operator()(const Measurement& lhs, const Measurement& rhs);

            protected:
                Eigen::VectorXd m_measurement;
                Eigen::MatrixXd m_covariance;

                std::vector<int> m_update; //!< Defines which variables should actually get passed to the filter.

                double m_time; //!< The real-valued time, in seconds

                double m_mahalanobis_threshold; //!< The mahalanobis distance threshold in number of sigmas

                Eigen::VectorXd m_latest_control; //!< The most recent control vector

                double m_latest_control_time; //!< The time stamp of the most recent control term

            };
        }
    }
}

#endif //MEASUREMENT_HPP
