#include "pid/linear_digital_filter.hpp"

#include <assert.h>
#include <math.h>

namespace rip
{
    namespace pid
    {
        LinearDigitalFilter::LinearDigitalFilter(PidInput* input,
                const std::vector<double>& ff_gains, const std::vector<double>& fb_gains)
            : Filter(input)
            , m_inputs(ff_gains.size())
            , m_outputs(fb_gains.size())
            , m_input_gains(ff_gains)
            , m_output_gains(fb_gains)
        {
        }

        LinearDigitalFilter LinearDigitalFilter::singlePoleIIR(PidInput* input, double time_constant, double period)
        {
            double gain = std::exp(-period / time_constant);
            return LinearDigitalFilter(std::move(input), {1.0 - gain}, { -gain});
        }

        LinearDigitalFilter LinearDigitalFilter::highPass(PidInput* input, double time_constant, double period)
        {
            double gain = std::exp(-period / time_constant);
            return LinearDigitalFilter(std::move(input), {gain, -gain}, { -gain});
        }

        LinearDigitalFilter LinearDigitalFilter::movingAverage(PidInput* input, int taps)
        {
            assert(taps > 0);

            std::vector<double> gains(taps, 1.0 / taps);
            return LinearDigitalFilter(std::move(input), gains, {});
        }

        double LinearDigitalFilter::get()
        {
            double rv = 0.0;

            // Rotate the inputs
            m_inputs.push_front(m_input->get());

            // Calculate the new value
            for (size_t i = 0; i < m_input_gains.size(); i++)
            {
                rv += m_inputs[i] * m_input_gains[i];
            }
            for (size_t i = 0; i < m_output_gains.size(); i++)
            {
                rv -= m_outputs[i] * m_output_gains[i];
            }

            // rotate the outputs
            m_outputs.push_front(rv);

            return rv;
        }
    }
}
