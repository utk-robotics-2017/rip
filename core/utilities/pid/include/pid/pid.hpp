#ifndef PID_HPP
#define PID_HPP

#include <chrono>
#include <tuple>

namespace rip
{
    namespace utilities
    {
        /**
         * A Propotional, Integral, Derivative Controller
         */
        class PidController
        {
        public:
            /**
             * Constructor
             *
             * @param p proporitional constant (K_p)
             * @param i integral constant (K_i)
             * @param d derivative constant (K_d)
             */
            PidController(const double p = 0.0, const double i = 0.0, const double d = 0.0) :
                m_p(p), m_i(i), m_d(d), m_target(0), m_last_error(0), m_accum_error(0),
                m_last_time(std::chrono::high_resolution_clock::now()) {}

            /**
             * setParams
             *
             * @param p proporitional constant (K_p)
             * @param i integral constant (K_i)
             * @param d derivative constant (K_d)
             */
            void setParams(const double p, const double i, const double d);

            /**
             * Constructor
             *
             * @return A tuple of the three PID parameters in order (p, i, d)
             */
            std::tuple<double, double, double> getParams();


            /**
             * setTarget
             *
             * @param target The target value for PID process
             */
            void setTarget(double target);

            /**
             * getTarget
             *
             * @return The target setpoint as a double
             */
            double getTarget();

            /**
             * update
             *
             * @brief The main update loop for the PID
             * @param val The input value for the PID used for error calculation
             * @return The updated output control signal
             */
            double update(double val);

        private:
            double m_p;
            double m_i;
            double m_d;
            double m_target;
            double m_last_error;
            double m_accum_error;
            std::chrono::high_resolution_clock::time_point m_last_time;

        }; // class PidController

    } // namespace utilities
} // namespace rip

#endif
