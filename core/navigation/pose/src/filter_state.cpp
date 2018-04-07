#include "pose/filter_state.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pose
        {
            FilterState::FilterState()
                : m_state()
                , m_estimate_error_covariance()
                , m_latest_control()
                , m_last_measurement_time(0.0)
                , m_last_control_time(0.0)
            {
            }

            Eigen::VectorXd & FilterState::state()
            {
                return m_state;
            }

            const Eigen::VectorXd& FilterState::state() const
            {
                return m_state;
            }

            Eigen::MatrixXd& FilterState::errorCovariance()
            {
                return m_estimate_error_covariance;
            }

            const Eigen::MatrixXd& FilterState::errorCovariance() const
            {
                return m_estimate_error_covariance;
            }

            Eigen::VectorXd& FilterState::latestControl()
            {
                return m_latest_control;
            }

            const Eigen::VectorXd & FilterState::latestControl() const
            {
                return m_latest_control;
            }

            double FilterState::lastMeasurementTime() const
            {
                return m_last_measurement_time;
            }

            double FilterState::lastControlTime() const
            {
                return m_last_control_time;
            }

            bool FilterState::operator()(const FilterState& lhs, const FilterState& rhs)
            {
                return lhs.lastMeasurementTime() < rhs.lastMeasurementTime();
            }
        }
    }
}