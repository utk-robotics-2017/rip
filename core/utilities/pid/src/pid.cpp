#include "pid/pid.hpp"

namespace rip
{
    namespace utilities
    {

        void PidController::setParams(double p, double i, double d)
        {
            m_p = p;
            m_i = i;
            m_d = d;
        }

        std::tuple<double, double, double> PidController::getParams()
        {
            return std::make_tuple(m_p, m_i, m_d);
        }

        double PidController::getTarget()
        {
            return m_target;
        }

        void PidController::setTarget(double target)
        {
            m_target = target;
        }

        double PidController::update(double val)
        {
            // Calculate the change in time since the last update loop
            std::chrono::high_resolution_clock::time_point time_now = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> dt = time_now - m_last_time;
            m_last_time = time_now;

            // Calculate the error
            double error = m_target - val;

            // Proportional Term (K_p * e(t))
            double p_term = m_p * error;

            // Integral Term (K_i * [e(t) dt from 0 to t])
            m_accum_error += error;
            double i_term = m_i * m_accum_error;

            // Derivative Term (K_d * de(t)/dt)
            double d_term = m_d * ((error - m_last_error) / dt.count());
            m_last_error = error;

            // Return the sum of the PID components
            return p_term + i_term + d_term;
        }

        void PidController::reset()
        {
            m_accum_error = 0;
            m_last_error = 0;
            m_last_time = std::chrono::high_resolution_clock::now();
        }
    }
}
