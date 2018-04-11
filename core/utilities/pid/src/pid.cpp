#include "pid/pid.hpp"

#include <misc/logger.hpp>

#include <cmath>

namespace rip
{
    namespace pid
    {

        template <typename T>
        T clamp(T value, T min, T max)
        {
            return std::max(min, std::min(max, value));
        }

        PidController::PidController(PidInput* input, PidOutput* output, const double p, const double i, const double d, const double f)
            : m_original_input(input)
            , m_output(output)
            , m_p(p)
            , m_i(i)
            , m_d(d)
            , m_f(f)
            , m_setpoint(0)
            , m_minimum_input(-1.0)
            , m_maximum_input(1.0)
            , m_input_range(2.0)
            , m_minimum_output(-1.0)
            , m_maximum_output(1.0)
            , m_previous_error(0)
            , m_total_error(0)
            , m_last_time(std::chrono::high_resolution_clock::now())
            , m_setpoint_time(std::chrono::high_resolution_clock::now())
        {
            m_filter = LinearDigitalFilter::movingAverage(m_original_input, 1);
            m_input = &m_filter;
        }

        void PidController::setGains(const double p, const double i, const double d)
        {
            m_p = p;
            m_i = i;
            m_d = d;
        }

        void PidController::setGains(const double p, const double i, const double d, const double f)
        {
            m_p = p;
            m_i = i;
            m_d = d;
            m_f = f;
        }

        std::tuple<double, double, double, double> PidController::gains() const
        {
            return std::make_tuple(m_p, m_i, m_d, m_f);
        }

        double PidController::setpoint() const
        {
            return m_setpoint;
        }

        double PidController::get() const
        {
            return m_result;
        }

        double PidController::deltaSetpoint() const
        {
            std::chrono::high_resolution_clock::time_point time_now = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> dt = time_now - m_setpoint_time;
            return (m_setpoint - m_previous_setpoint) / dt.count();
        }

        double PidController::error() const
        {
            return m_error;
        }

        void PidController::setSetpoint(double setpoint)
        {
            m_setpoint = clamp(setpoint, m_minimum_input, m_maximum_input);
        }

        bool PidController::onTarget() const
        {
            double err = error();
            switch (m_tolerance_type)
            {
            case ToleranceType::kPercent:
                misc::Logger::getInstance()->debug("Error: {}, Tol: {}", err, m_tolerance/ 100.0 * m_input_range);
                return std::fabs(err) < m_tolerance / 100.0 * m_input_range;
            case ToleranceType::kAbsolute:
                misc::Logger::getInstance()->debug("Error: {}, Tol: {}", err, m_tolerance)  ;
                return std::fabs(err) < m_tolerance;
            case ToleranceType::kNoTolerance:
                return false;
            default:
                assert(false);
            }
            return false;
        }

        void PidController::setContinuous(bool continuous)
        {
            m_continuous = continuous;
        }

        void PidController::setInputRange(double min, double max)
        {
            m_minimum_input = min;
            m_maximum_input = max;
            m_input_range = max - min;
            setSetpoint(m_setpoint);
        }

        void PidController::setOutputRange(double min, double max)
        {
            m_minimum_output = min;
            m_maximum_output = max;
        }

        void PidController::enable()
        {
            m_enabled = true;
        }

        void PidController::disable()
        {
            m_enabled = false;
            m_output->set(0);
        }

        bool PidController::isEnabled() const
        {
            return m_enabled;
        }


        /**
         * Set the percentage error which is considered tolerable
         * for use with @{PidController#onTarget}
         */
        void PidController::setTolerance(double percent)
        {
            m_tolerance_type = ToleranceType::kPercent;
            m_tolerance = percent;
        }

        /**
         * Set the absolute error which is considered tolerable
         * for use with @{PidController#onTarget}
         */
        void PidController::setAbsoluteTolerance(double value)
        {
            m_tolerance_type = ToleranceType::kAbsolute;
            m_tolerance = value;
        }

        /**
         * Set the percentage error which is considered tolerable
         * for use with @{PidController#onTarget}
         */
        void PidController::setPercentTolerance(double percent)
        {
            m_tolerance_type = ToleranceType::kPercent;
            m_tolerance = percent;
        }

        void PidController::calculate()
        {
            if (m_original_input == nullptr || m_output == nullptr)
            {

                misc::Logger::getInstance()->debug_if(m_original_input == nullptr, "Input null");
                misc::Logger::getInstance()->debug_if(m_output == nullptr, "Output null");
                return;
            }

            if (m_enabled)
            {
                std::chrono::high_resolution_clock::time_point time_now = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double, std::milli> dt = time_now - m_last_time;

                double input = m_input->get();
                misc::Logger::getInstance()->debug("Input: {}", input);

                PidInput::Type type = m_input->type();

                // Storage for function inputs
                m_f = calculateFeedForward();

                misc::Logger::getInstance()->debug("F: {}", m_f);


                // Storage for function input-outputs
                double error = continuousError(m_setpoint - input);
                misc::Logger::getInstance()->debug("Error: {}", error);
                if (type == PidInput::Type::kRate)
                {
                    if (m_p != 0)
                    {
                        m_total_error = clamp(m_total_error + error, m_minimum_output / m_p, m_maximum_output / m_p);
                    }

                    m_result = m_d * error + m_p * m_total_error + m_f;
                }
                else
                {
                    if (m_i != 0)
                    {
                        m_total_error = clamp(m_total_error + error, m_minimum_output / m_i, m_maximum_output / m_i);
                    }

                    m_result = m_p * error + m_i * m_total_error + m_d * (error - m_previous_error)  / (dt.count() / 1000.0) + m_f;
                }

                m_result = clamp(m_result, m_minimum_output, m_maximum_output);
                misc::Logger::getInstance()->debug("Result: {}", m_result);
                if (m_enabled)
                {
                    m_output->set(m_result);
                }

                m_last_time = time_now;
                m_previous_error = m_error;
                m_error = error;
            }
        }

        double PidController::calculateFeedForward()
        {
            if (m_input->type() == PidInput::Type::kRate)
            {
                return m_f * setpoint();
            }
            else
            {
                double temp = m_f * deltaSetpoint();
                m_previous_setpoint = m_setpoint;
                m_setpoint_time = std::chrono::high_resolution_clock::now();
                return temp;
            }
        }

        double PidController::continuousError(double error) const
        {
            if (m_continuous && m_input_range != 0)
            {
                error = std::fmod(error, m_input_range);
                if (std::fabs(error) > m_input_range / 2)
                {
                    if (error > 0)
                    {
                        return error - m_input_range;
                    }
                    else
                    {
                        return error + m_input_range;
                    }
                }
            }
            return error;
        }

        void PidController::reset()
        {
            disable();
            m_total_error = 0;
            m_previous_error = 0;
            m_result = 0;
            m_last_time = std::chrono::high_resolution_clock::now();
            m_setpoint_time = std::chrono::high_resolution_clock::now();
        }
    }
}
