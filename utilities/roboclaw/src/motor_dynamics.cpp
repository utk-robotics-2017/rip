#include "motor_dynamics.hpp"

#include "exceptions.hpp"

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
        m_acceleration = std::make_shared<units::Acceleration>(acceleration());
    }

    std::shared_ptr<units::Acceleration> MotorDynamics::getAcceleration() const
    {
        return m_acceleration;
    }

    void MotorDynamics::setDeceleration(units::Acceleration deceleration)
    {
        m_deceleration = std::make_shared<units::Acceleration>(deceleration());
    }

    std::shared_ptr<units::Acceleration> MotorDynamics::getDeceleration() const
    {
        return m_deceleration;
    }

    MotorDynamics::DType MotorDynamics::getDType() const
    {
        DType d_type = static_cast<DType>(static_cast<uint8_t>(static_cast<bool>(m_speed)) << 3 | static_cast<uint8_t>(static_cast<bool>(m_acceleration)) << 2 | static_cast<uint8_t>(static_cast<bool>(m_deceleration)) << 1 | static_cast<uint8_t>(static_cast<bool>(m_distance)));
        switch(d_type)
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