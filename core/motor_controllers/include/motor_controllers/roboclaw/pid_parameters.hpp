/*
 * The RIP License (Revision 0.3):
 * This software is available without warranty and without support.
 * Use at your own risk. Literally. It might delete your filesystem or
 * eat your cat. As long as you retain this notice, you can do whatever
 * you want with this. If we meet some day, you owe me a beer.
 *
 * Go Vols!
 *
 *  __    __  ________  __    __        _______   ______  _______
 * |  \  |  \|        \|  \  /  \      |       \ |      \|       \
 * | $$  | $$ \$$$$$$$$| $$ /  $$      | $$$$$$$\ \$$$$$$| $$$$$$$\
 * | $$  | $$   | $$   | $$/  $$       | $$__| $$  | $$  | $$__/ $$
 * | $$  | $$   | $$   | $$  $$        | $$    $$  | $$  | $$    $$
 * | $$  | $$   | $$   | $$$$$\        | $$$$$$$\  | $$  | $$$$$$$
 * | $$__/ $$   | $$   | $$ \$$\       | $$  | $$ _| $$_ | $$
 *  \$$    $$   | $$   | $$  \$$\      | $$  | $$|   $$ \| $$
 *   \$$$$$$     \$$    \$$   \$$       \$$   \$$ \$$$$$$ \$$
 */
#ifndef PID_PARAMETERS_HPP
#define PID_PARAMETERS_HPP

#include <stdint.h>

namespace rip
{
    namespace motorcontrollers
    {
        namespace roboclaw
        {
            /**
             * @struct PIDParameters
             * @brief The parameters for PID
             */

            struct VelocityPIDParameters
            {
                float kp = static_cast<float>(0x10000) / 65536; //< Proportional Gain
                float ki = static_cast<float>(0x8000) / 65536; //< Integral Gain
                float kd = static_cast<float>(0x4000) / 65536; //< Derivative Gain
                uint32_t qpps = 44000; //< Quadrature Pulses Per Second - Maximum number of ticks per second
            }; // struct VelocityPIDParameters

            struct PositionPIDParameters
            {
                float kp = 0;
                float ki = 0;
                float kd = 0;
                uint32_t kiMax = 0; //< Maximum Integral Windup
                uint32_t deadzone = 0; //< Deadzone in encoder counts
                uint32_t min = 0; //< Minimum Position
                uint32_t max = 0; //< Maximum Position
            }; // struct PositionPIDParameters
        } // namespace roboclaw
    }
}
#endif // PID_PARAMETERS_HPP
