#ifndef PID_OUTPUT_HPP
#define PID_OUTPUT_HPP

namespace rip
{
    namespace pid
    {
        /**
     * An interface for the generic output for the PID.
     *
     *
     */
        class PidOutput
        {
        public:
            virtual void set(double output) = 0;
        };
    }
}

#endif // PID_OUTPUT_HPP
