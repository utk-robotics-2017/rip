#ifndef LINEAR_DIGITAL_FILTER_HPP
#define LINEAR_DIGITAL_FILTER_HPP

#include <memory>
#include <vector>

#include "filter.hpp"
#include "pid_input.hpp"
#include "circular_buffer.hpp"

namespace rip
{
    namespace pid
    {
        class LinearDigitalFilter : public Filter
        {
        public:
            LinearDigitalFilter(PidInput* source, const std::vector<double>& fb_gains);
            LinearDigitalFilter(PidInput* source, const std::vector<double>& ff_gains, const std::vector<double>& fb_gains);

            // Common filers
            /**
             * Creates a one-pole IIR low-pass filter of the form:
             * 	y[n] = (1 - gain) * x[n] + gain * y[n-1]
             * where gain = e<sup>-dt / T</sup>
             *
             * This filter is stable for time constants greater than zero
             * @param  source        The input
             * @param  time_constant The discrete time constant
             * @param  period        The period
             */
            static LinearDigitalFilter singlePoleIIR(PidInput* source, double time_constant, double period);

            /**
             * [highPass description]
             * @param  source        [description]
             * @param  time_constant [description]
             * @param  period        [description]
             * @return               [description]
             */
            static LinearDigitalFilter highPass(PidInput* source, double time_constant, double period);
            static LinearDigitalFilter movingAverage(PidInput* source, int taps);

            /**
             * Calculates the next value of the filter
             */
            double get() override;

            void reset() override;

        private:
            CircularBuffer<double> m_inputs;
            CircularBuffer<double> m_outputs;
            std::vector<double> m_input_gains;
            std::vector<double> m_output_gains;
        };
    }
}

#endif // LINEAR_DIGITAL_FILTER_HPP
