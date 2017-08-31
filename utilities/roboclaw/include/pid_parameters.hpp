#ifndef PID_PARAMETERS_HPP
#define PID_PARAMETERS_HPP

#include <stdint.h>

namespace roboclaw
{
    /**
     * @struct PIDParameters
     * @brief The parameters for PID
     */
    struct PIDParameters
    {
        float kp; //< Proportional Gain
        float ki; //< Integral Gain
        float kd; //< Derivative Gain
    }; // struct PIDParameters


    struct VelocityPIDParameters : public PIDParameters
    {
        uint32_t qpps; //< Quadrature Pulses Per Second - Maximum number of ticks per second
    }; // struct VelcityPIDParameters

    struct PositionPIDParameters : public PIDParameters
    {
        uint32_t kiMax; //< Maximum Integral Windup
        uint32_t deadzone; //< Deadzone in encoder counts
        uint32_t min; //< Minimum Position
        uint32_t max; //< Maximum Position
    }; // struct PositionPIDParameters
} // namespace roboclaw

#endif // PID_PARAMETERS_HPP
