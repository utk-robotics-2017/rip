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
#include "motor_dynamics.hpp"

#include "exceptions.hpp"

namespace rip
{
    namespace utilities
    {
        namespace roboclaw
        {
            MotorDynamics::MotorDynamics()
                : m_distance(nullptr)
                , m_speed(nullptr)
                , m_acceleration(nullptr)
                , m_deceleration(nullptr)
            {}

            void MotorDynamics::setDistance(units::Distance distance)
            {
                if(distance() < 0.0)
                {
                    throw OutOfRange("Distance should be a positive value.");
                }
                m_distance = std::make_shared<units::Distance>(distance());
            }

            std::shared_ptr<units::Distance> MotorDynamics::getDistance() const
            {
                return m_distance;
            }

            void MotorDynamics::setSpeed(units::Velocity speed)
            {
                m_speed = std::make_shared<units::Velocity>(speed());
            }

            std::shared_ptr<units::Velocity> MotorDynamics::getSpeed() const
            {
                return m_speed;
            }

            void MotorDynamics::setAcceleration(units::Acceleration acceleration)
            {
                if(acceleration() < 0.0)
                {
                    throw OutOfRange("Acceleration should be a positive value.");
                }
                m_acceleration = std::make_shared<units::Acceleration>(acceleration());
            }

            std::shared_ptr<units::Acceleration> MotorDynamics::getAcceleration() const
            {
                return m_acceleration;
            }

            void MotorDynamics::setDeceleration(units::Acceleration deceleration)
            {
                if(deceleration() < 0.0)
                {
                    throw OutOfRange("deceleration should be a positive value.");
                }
                m_deceleration = std::make_shared<units::Acceleration>(deceleration());
            }

            std::shared_ptr<units::Acceleration> MotorDynamics::getDeceleration() const
            {
                return m_deceleration;
            }

            MotorDynamics::DType MotorDynamics::getDType() const
            {

                DType d_type = static_cast<DType>(static_cast<uint8_t>(static_cast<bool>(m_speed)) << 3 | static_cast<uint8_t>(static_cast<bool>(m_acceleration)) << 2 | static_cast<uint8_t>(static_cast<bool>(m_deceleration)) << 1 | static_cast<uint8_t>(static_cast<bool>(m_distance)));
				// std::cout << "D type debugging: " << std::hex << (static_cast<uint8_t>(static_cast<bool>(m_speed)) << 3 | static_cast<uint8_t>(static_cast<bool>(m_acceleration)) << 2 | static_cast<uint8_t>(static_cast<bool>(m_deceleration)) << 1 | static_cast<uint8_t>(static_cast<bool>(m_distance))) << std::endl;
                switch (d_type)
                {
                    case DType::kNone:
                    case DType::kSpeed:
                    case DType::kSpeedAccel:
                    case DType::kSpeedAccelDist:
                    case DType::kSpeedDist:
                    case DType::kSpeedAccelDecelDist:
                        break;
                    default:
                        throw UnknownDType();
                }

                return d_type;
            }
        }
    }
}
