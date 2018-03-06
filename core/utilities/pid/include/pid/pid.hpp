#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <chrono>
#include <tuple>
#include <memory>

#include "pid/pid_input.hpp"
#include "pid/pid_output.hpp"
#include "pid/linear_digital_filter.hpp"

namespace rip
{
    namespace pid
    {
        /**
         * A Propotional, Integral, Derivative Controller
         *
         * Creates a separate thread that reads the given PidInput
         * and takes care of the integral calculations, as well as
         * setting the given PidOutput.
         */
        class PidController
        {
        public:
            /**
             * Constructor
             *
             * @param p Proportional constant (K_p)
             * @param i Integral constant (K_i)
             * @param d Derivative constant (K_d)
             */
            PidController(PidInput* input, PidOutput* output, const double p, const double i = 0.0, const double d = 0.0, const double f = 0.0) ;

            /**
             * setParams
             *
             * @param p Proportional constant (K_p)
             * @param i Integral constant (K_i)
             * @param d Derivative constant (K_d)
             */
            void setGains(const double p, const double i, const double d);

            /**
             * Sets the PIDF gains
             * @param p Proportional constant (K_p)
             * @param i Integral constant (K_i)
             * @param d Derivative constant (K_d)
             * @param f Feed forward constant (K_f)
             */
            void setGains(const double p, const double i, const double d, const double f);

            /**
             * getParams
             *
             * @return A tuple of the PID parameters in order (p, i, d, f)
             */
            std::tuple<double, double, double, double> gains() const;

            /**
             * Sets the setpoint for the PID
             */
            void setSetpoint(double setpoint);

            /**
             * Returns the setpoint for the PID
             */
            double setpoint() const;

            /**
             * Returns the change in setpoint
             * Used for the feed forward
             */
            double deltaSetpoint() const;

            /**
             * Returns the most recent error between the setpoint and the input
             */
            double error() const;

            /**
             * Return the current PID result
             *
             * This is always centered on zero and constrained to the max and min outputs
             */
            double get() const;

            /**
             * Returns whether the PID is on target
             */
            bool onTarget() const;

            /**
             * Set the PID controller to consider the input to be continuous
             *
             * Rather than using the max and min input as constraints, it considers them
             * to be the same point and automatically calculates the shortest route to
             * the setpoint.
             *
             */
            void setContinuous(bool continuous);

            /**
             * Sets the maximum and minimum values expected from the input
             */
            void setInputRange(double min, double max);

            /**
             * Sets the maximum and minimum values expected for the output
             */
            void setOutputRange(double min, double max);

            /**
             * Enable the PID
             */
            void enable();

            /**
             * Disable the PID
             */
            void disable();

            /**
             * Returns whether the PID is enabled
             */
            bool isEnabled() const;

            /**
             * Set the percentage tolerance for whether the PID is on target
             */
            void setTolerance(double percent);

            /**
             * Set the absolute tolerance for whether the PID is on target
             */
            void setAbsoluteTolerance(double value);

            /**
             * Set the percentage tolerance for whether the PID is on target
             */
            void setPercentTolerance(double percent);

            /**
             * reset
             *
             * @brief Resets the internal error calculation
             */
            void reset();

            /**
             * Read the input, calculate the output accordingly, and write to the output
             */
            void calculate();

        private:

            /**
             * Calculate the feed forward
             *
             * Both of the provided feed forward calculations are velocity feed forwards.
             *
             * If a velocity PID controller is being used, the f term should be set to 1
             * over the maximum setpoint for the output. If a position PID controller is
             * being used, the F term should be set to 1 over the maximum speed for the
             * output measured in setpoint units per this controller's update period.
             */
            double calculateFeedForward();
            double continuousError(double error) const;

            double m_p;
            double m_i;
            double m_d;
            double m_f;

            double m_setpoint;
            double m_previous_setpoint;
            std::chrono::high_resolution_clock::time_point m_setpoint_time;

            double m_error;
            double m_result;

            PidInput* m_original_input;
            PidInput* m_input;
            LinearDigitalFilter m_filter{nullptr, {}, {}};
            PidOutput* m_output;

            bool m_continuous;
            double m_minimum_input;
            double m_maximum_input;
            double m_input_range;
            double m_minimum_output;
            double m_maximum_output;

            bool m_enabled;

            double m_previous_error;
            double m_total_error;

            enum class ToleranceType
            {
                kAbsolute,
                kPercent,
                kNoTolerance
            };
            ToleranceType m_tolerance_type = ToleranceType::kNoTolerance;
            double m_tolerance = 0.05;

            std::chrono::high_resolution_clock::time_point m_last_time;

        }; // class PidController

    } // namespace pid
} // namespace rip

#endif // PID_CONTROLLER_HPP
