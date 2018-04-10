#include "pose/measurement.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pose
        {
            Measurement::Measurement()
                : m_latest_control()
                , m_latest_control_time(0.0)
                , m_time(0.0)
                , m_mahalanobis_threshold(std::numeric_limits<double>::max())
            {
            }

            Eigen::VectorXd& Measurement::measurement()
            {
                return m_measurement;
            }

            const Eigen::VectorXd& Measurement::measurement() const
            {
                return m_measurement;
            }

            Eigen::MatrixXd& Measurement::covariance()
            {
                return m_covariance;
            }

            const Eigen::MatrixXd& Measurement::covariance() const
            {
                return m_covariance;
            }

            std::vector<int>& Measurement::updateVector()
            {
                return m_update;
            }

            const std::vector<int>& Measurement::updateVector() const
            {
                return m_update;
            }

            double& Measurement::time()
            {
                return m_time;
            }

            double Measurement::time() const
            {
                return m_time;
            }

            double& Measurement::mahalanobisThreshold()
            {
                return m_mahalanobis_threshold;
            }

            double Measurement::mahalanobisThreshold() const
            {
                return m_mahalanobis_threshold;
            }

            Eigen::VectorXd& Measurement::lastControl()
            {
                return m_latest_control;
            }

            const Eigen::VectorXd& Measurement::lastControl() const
            {
                return m_latest_control;
            }

            double& Measurement::lastControlTime()
            {
                return m_latest_control_time;
            }

            double Measurement::lastControlTime() const
            {
                return m_latest_control_time;
            }

            bool Measurement::operator()(const std::shared_ptr<Measurement>& lhs, const std::shared_ptr<Measurement>& rhs)
            {
                return (*this)(*(lhs.get()), *(rhs.get()));
            }

            bool Measurement::operator()(const Measurement& lhs, const Measurement& rhs)
            {
                return lhs.time() > rhs.time();
            }
        }
    }
}