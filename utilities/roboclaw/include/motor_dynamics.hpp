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
#ifndef MOTOR_DYNAMICS_HPP
#define MOTOR_DYNAMICS_HPP

#include <memory>

#include <units.hpp>

namespace rip
{
    namespace utilities
    {
        namespace roboclaw
        {
            /**
             * @struct MotorDynamics
             * @brief The dynamic properties for the motor
             */
            class MotorDynamics
            {
            public:
                /**
                 * @brief Constructor
                 */
                MotorDynamics();

                /**
                 * @brief Sets the distance for the motor dynamics
                 * @param distance
                 */
                void setDistance(units::Distance distance);

                /**
                 * @brief Returns the distance (nullptr if distance isn't set)
                 * @returns The distance part of the motor dynamics
                 */
                std::shared_ptr<units::Distance> getDistance() const;

                /**
                 * @brief Sets the speed
                 * @param speed
                 */
                void setSpeed(units::Velocity speed);

                /**
                 * @brief Returns the speed (nullptr if speed isn't set)
                 * @returns
                 */
                std::shared_ptr<units::Velocity> getSpeed() const;

                /**
                 * @brief Sets the acceleration
                 * @param acceleration
                 */
                void setAcceleration(units::Acceleration acceleration);

                /**
                 * @brief Returns the acceleration (nullptr if acceleration isn't set)
                 * @returns
                 */
                std::shared_ptr<units::Acceleration> getAcceleration() const;

                /**
                 * @brief Sets the deceleration
                 * @param deceleration
                 */
                void setDeceleration(units::Acceleration deceleration);

                /**
                 * @brief Returns the deceleration (nullptr if deceleration isn't set)
                 * @return
                 */
                std::shared_ptr<units::Acceleration> getDeceleration() const;

                enum class DType
                {
                    kNone                = 0x0,
                    kSpeed               = 0x8,
                    kSpeedAccel          = 0xC,
                    kSpeedDist           = 0x9,
                    kSpeedAccelDist      = 0xD,
                    kSpeedAccelDecelDist = 0xF
                };

                DType getDType() const;

            private:
                std::shared_ptr<units::Distance> m_distance;
                std::shared_ptr<units::Velocity> m_speed;
                std::shared_ptr<units::Acceleration> m_acceleration;
                std::shared_ptr<units::Acceleration> m_deceleration;
            }; // struct MotorDynamics
        } // namespace roboclaw
    }
}
#endif // MOTOR_DYNAMICS_HPP
