#include "drivetrains/two_roboclaw_appendage_drivetrain.hpp"
#include <drivetrains/exceptions.hpp>
#include <appendages/roboclaw.hpp>
#include <cmath>
#include <fmt/format.h>
#include <iostream>

namespace rip
{
    namespace navigation
    {
        namespace drivetrains
        {
            TwoRoboclawAppendageDrivetrain::TwoRoboclawAppendageDrivetrain(const std::string& name,
                std::shared_ptr<Roboclaw> left,
                std::shared_ptr<Roboclaw> right,
                double ticks_per_rev,
                units::Distance wheel_radius,
                std::shared_ptr<NavX> navx)
                : Drivetrain(name)
                , m_left(left)
                , m_right(right)
                , m_navx(navx)
                , m_ticks_per_rev(ticks_per_rev)
                , m_wheel_radius(wheel_radius)
            {}

            TwoRoboclawAppendageDrivetrain::~TwoRoboclawAppendageDrivetrain()
            {
                stop();
            }

            void TwoRoboclawAppendageDrivetrain::drive(double power)
            {
                if (std::abs(power) > 1)
                {
                    throw OutOfRangeException("out of range");
                }
                m_left->setM1M2Duty(static_cast<int>(32767 * power), static_cast<int>(32767 * power));
                m_right->setM1M2Duty(static_cast<int>(32767 * power), static_cast<int>(32767 * power));
            }

            void TwoRoboclawAppendageDrivetrain::drive(double left, double right)
            {
                if (std::abs(left) > 1 || std::abs(right) > 1)
                {
                    throw OutOfRangeException("out of range");
                }
                m_left->setM1M2Duty(static_cast<int>(32767 * left), static_cast<int>(32767 * left));
                m_right->setM1M2Duty(static_cast<int>(32767 * right), static_cast<int>(32767 * right));
            }

            void TwoRoboclawAppendageDrivetrain::drive(double front_left, double front_right, double back_left,
                double back_right)
            {
                if (std::abs(front_left) > 1 || std::abs(front_right) > 1 || std::abs(back_left) > 1 ||
                std::abs(back_right) > 1)
                {
                    throw OutOfRangeException("out of range");
                }

                m_left->setM1Duty(static_cast<int>(32767 * front_left));
                m_right->setM1Duty(static_cast<int>(32767 * front_right));

                m_left->setM2Duty(static_cast<int>(32767 * back_left));
                m_right->setM2Duty(static_cast<int>(32767 * back_right));
            }

            void TwoRoboclawAppendageDrivetrain::drive(const MotorDynamics& command)
            {
                setDynamics(command);
            }

            void TwoRoboclawAppendageDrivetrain::drive(const MotorDynamics& left, const MotorDynamics& right)
            {
                setDynamics(Motor::kFrontLeft, left);
                setDynamics(Motor::kBackLeft, left);

                setDynamics(Motor::kFrontRight, right);
                setDynamics(Motor::kBackRight, right);
            }

            void TwoRoboclawAppendageDrivetrain::drive(const MotorDynamics& front_left, const MotorDynamics& front_right,
                 const MotorDynamics& back_left, const MotorDynamics& back_right)
            {
                setDynamics(Motor::kFrontLeft, front_left);
                setDynamics(Motor::kBackLeft, back_left);

                setDynamics(Motor::kFrontRight, front_right);
                setDynamics(Motor::kBackRight, back_right);
            }

            std::vector<units::Distance> TwoRoboclawAppendageDrivetrain::readEncoders(const std::vector<Motor>& motors)
            {
                std::vector<units::Distance> data;
                for(uint i=0; i<motors.size(); i++)
                {
                    switch(motors[i])
                    {
                        case Motor::kFrontLeft:
                        {
                            data.push_back(static_cast<double>(m_left->readM1Encoder()) / m_ticks_per_rev * m_wheel_radius * M_PI * 2);
                            break;
                        }
                        case Motor::kFrontRight:
                        {
                            data.push_back(static_cast<double>(m_right->readM1Encoder()) / m_ticks_per_rev * m_wheel_radius * M_PI * 2);
                            break;
                        }
                        case Motor::kBackLeft:
                        {
                            data.push_back(static_cast<double>(m_left->readM2Encoder()) / m_ticks_per_rev * m_wheel_radius * M_PI * 2);
                            break;
                        }
                        case Motor::kBackRight:
                        {
                            data.push_back(static_cast<double>(m_right->readM2Encoder()) / m_ticks_per_rev * m_wheel_radius * M_PI * 2);
                            break;
                        }
                        default:
                        {
                            throw InvalidMotorException(fmt::format("Invalid motor, parameter {}", i+1));
                        }
                    }
                }
                return data;
            }

            units::Distance TwoRoboclawAppendageDrivetrain::readEncoder(const Motor& motor)
            {
                switch(motor)
                {
                    case Motor::kFrontLeft:
                    {
                        return static_cast<double>(m_left->readM1Encoder()) / m_ticks_per_rev * m_wheel_radius * M_PI * 2;
                    }
                    case Motor::kFrontRight:
                    {
                        return static_cast<double>(m_right->readM1Encoder()) / m_ticks_per_rev * m_wheel_radius * M_PI * 2;
                    }
                    case Motor::kBackLeft:
                    {
                        return static_cast<double>(m_left->readM2Encoder()) / m_ticks_per_rev * m_wheel_radius * M_PI * 2;
                    }
                    case Motor::kBackRight:
                    {
                        return static_cast<double>(m_right->readM2Encoder()) / m_ticks_per_rev * m_wheel_radius * M_PI * 2;
                    }
                    default:
                    {
                        throw InvalidMotorException("Invalid motor");
                    }
                }
            }

            std::vector<units::Velocity> TwoRoboclawAppendageDrivetrain::readEncoderVelocities(
                const std::vector<Motor>& motors)
            {
                std::vector<units::Velocity> data;
                for(uint i=0; i<motors.size(); i++)
                {
                    switch(motors[i])
                    {
                        case Motor::kFrontLeft:
                        {
                            data.push_back(m_left->readM1EncoderSpeed() / m_ticks_per_rev * m_wheel_radius * M_PI * 2 / units::s);
                            break;
                        }
                        case Motor::kFrontRight:
                        {
                            data.push_back(m_right->readM1EncoderSpeed() / m_ticks_per_rev * m_wheel_radius * M_PI * 2 / units::s);
                            break;
                        }
                        case Motor::kBackLeft:
                        {
                            data.push_back(m_left->readM2EncoderSpeed() / m_ticks_per_rev * m_wheel_radius * M_PI * 2 / units::s);
                            break;
                        }
                        case Motor::kBackRight:
                        {
                            data.push_back(m_right->readM2EncoderSpeed() / m_ticks_per_rev * m_wheel_radius * M_PI * 2 / units::s);
                            break;
                        }
                        default:
                        {
                            throw InvalidMotorException(fmt::format("Invalid motor, parameter {}", i+1));
                        }
                    }
                }
                return data;
            }

            units::Velocity TwoRoboclawAppendageDrivetrain::readEncoderVelocity(const Motor& motor)
            {
                switch(motor)
                {
                    case Motor::kFrontLeft:
                    {
                        return m_left->readM1EncoderSpeed() / m_ticks_per_rev * m_wheel_radius * M_PI * 2 / units::s;
                    }
                    case Motor::kFrontRight:
                    {
                        return m_right->readM1EncoderSpeed() / m_ticks_per_rev * m_wheel_radius * M_PI * 2 / units::s;
                    }
                    case Motor::kBackLeft:
                    {
                        return m_left->readM2EncoderSpeed() / m_ticks_per_rev * m_wheel_radius * M_PI * 2 / units::s;
                    }
                    case Motor::kBackRight:
                    {
                        return m_right->readM2EncoderSpeed() / m_ticks_per_rev * m_wheel_radius * M_PI * 2 / units::s;
                    }
                    default:
                    {
                        throw InvalidMotorException("Invalid motor");
                    }
                }
            }

            void TwoRoboclawAppendageDrivetrain::setDynamics(const Motor& motor, const MotorDynamics& dynamics, bool respectBuffer)
            {
                int32_t speed;
                uint32_t accel, dist, decel;

                switch(dynamics.getDType())
                {
                    case MotorDynamics::DType::kNone:
                    {
                        return;
                    }
                    case MotorDynamics::DType::kSpeed:
                    {
                        speed = static_cast<int32_t>((*dynamics.getSpeed() / (m_wheel_radius * M_PI * 2)).to(1 / units::s) * m_ticks_per_rev);
                        switch(motor)
                        {
                            case Motor::kFrontLeft:
                            {
                                m_left->setM1Speed(speed);
                                break;
                            }
                            case Motor::kFrontRight:
                            {
                                m_right->setM1Speed(speed);
                                break;
                            }
                            case Motor::kBackLeft:
                            {
                                m_left->setM2Speed(speed);
                                break;
                            }
                            case Motor::kBackRight:
                            {
                                m_right->setM2Speed(speed);
                                break;
                            }
                            default:
                            {
                                throw InvalidMotorException("Invalid motor");
                            }
                        }
                        return;
                    }
                    case MotorDynamics::DType::kSpeedAccel:
                    {
                        speed = static_cast<int32_t>((*dynamics.getSpeed() / (m_wheel_radius * M_PI * 2)).to(1 / units::s) * m_ticks_per_rev);
                        accel = static_cast<uint32_t>((*dynamics.getAcceleration() / (m_wheel_radius * M_PI * 2)).to(1 / (units::s * units::s)) * m_ticks_per_rev);
                        switch(motor)
                        {
                            case Motor::kFrontLeft:
                            {
                                m_left->setM1SpeedAccel(accel, speed);
                                break;
                            }
                            case Motor::kFrontRight:
                            {
                                m_right->setM1SpeedAccel(accel, speed);
                                break;
                            }
                            case Motor::kBackLeft:
                            {
                                m_left->setM2SpeedAccel(accel, speed);
                                break;
                            }
                            case Motor::kBackRight:
                            {
                                m_right->setM2SpeedAccel(accel, speed);
                                break;
                            }
                            default:
                            {
                                throw InvalidMotorException("Invalid motor");
                            }
                        }
                        return;
                    }
                    case MotorDynamics::DType::kSpeedDist:
                    {
                        speed = static_cast<int32_t>((*dynamics.getSpeed() / (m_wheel_radius * M_PI * 2)).to(1 / units::s) * m_ticks_per_rev);
                        dist = static_cast<uint32_t>((*dynamics.getDistance() / (m_wheel_radius * M_PI * 2)).to(units::none) * m_ticks_per_rev);

                        switch(motor)
                        {
                            case Motor::kFrontLeft:
                            {
                                m_left->setM1SpeedDist(speed, dist, respectBuffer);
                                break;
                            }
                            case Motor::kFrontRight:
                            {
                                m_right->setM1SpeedDist(speed, dist, respectBuffer);
                                break;
                            }
                            case Motor::kBackLeft:
                            {
                                m_left->setM2SpeedDist(speed, dist, respectBuffer);
                                break;
                            }
                            case Motor::kBackRight:
                            {
                                m_right->setM2SpeedDist(speed, dist, respectBuffer);
                                break;
                            }
                            default:
                            {
                                throw InvalidMotorException("Invalid motor");
                            }
                        }
                        return;
                    }
                    case MotorDynamics::DType::kSpeedAccelDist:
                    {
                        speed = static_cast<int32_t>((*dynamics.getSpeed() / m_wheel_radius / (units::pi * 2))() * m_ticks_per_rev);
                        dist = static_cast<uint32_t>((*dynamics.getDistance() / m_wheel_radius / (units::pi * 2))() * m_ticks_per_rev);
                        accel = static_cast<uint32_t>((*dynamics.getAcceleration() / m_wheel_radius / (units::pi * 2))() * m_ticks_per_rev);

                        switch(motor)
                        {
                            case Motor::kFrontLeft:
                            {
                                m_left->setM1SpeedAccelDist(accel, speed, dist, respectBuffer);
                                break;
                            }
                            case Motor::kFrontRight:
                            {
                                m_right->setM1SpeedAccelDist(accel, speed, dist, respectBuffer);
                                break;
                            }
                            case Motor::kBackLeft:
                            {
                                m_left->setM2SpeedAccelDist(accel, speed, dist, respectBuffer);
                                break;
                            }
                            case Motor::kBackRight:
                            {
                                m_right->setM2SpeedAccelDist(accel, speed, dist, respectBuffer);
                                break;
                            }
                            default:
                            {
                                throw InvalidMotorException("Invalid motor");
                            }
                        }
                        return;
                    }
                    case MotorDynamics::DType::kSpeedAccelDecelDist:
                    {
                        speed = static_cast<int32_t>((*dynamics.getSpeed() / m_wheel_radius / (units::pi * 2))() * m_ticks_per_rev);
                        dist = static_cast<uint32_t>((*dynamics.getDistance() / m_wheel_radius / (units::pi * 2))() * m_ticks_per_rev);
                        accel = static_cast<uint32_t>((*dynamics.getAcceleration() / m_wheel_radius / (units::pi * 2))() * m_ticks_per_rev);
                        decel = static_cast<uint32_t>((*dynamics.getDeceleration() / m_wheel_radius / (units::pi * 2))() * m_ticks_per_rev);

                        switch(motor)
                        {
                            case Motor::kFrontLeft:
                            {
                                m_left->setM1SpeedAccelDecelDist(accel, speed, decel, dist, respectBuffer);
                                break;
                            }
                            case Motor::kFrontRight:
                            {
                                m_right->setM1SpeedAccelDecelDist(accel, speed, decel, dist, respectBuffer);
                                break;
                            }
                            case Motor::kBackLeft:
                            {
                                m_left->setM2SpeedAccelDecelDist(accel, speed, decel, dist, respectBuffer);
                                break;
                            }
                            case Motor::kBackRight:
                            {
                                m_right->setM2SpeedAccelDecelDist(accel, speed, decel, dist, respectBuffer);
                                break;
                            }
                            default:
                            {
                                throw InvalidMotorException("Invalid motor");
                            }
                        }
                        return;
                    }
                    default:
                    {
                        assert(false);
                    }
                }
            }

            void TwoRoboclawAppendageDrivetrain::setDynamics(const MotorDynamics& dynamics, bool respectBuffer)
            {
                int32_t speed;
                uint32_t accel, dist, decel;

                switch(dynamics.getDType())
                {
                    case MotorDynamics::DType::kNone:
                    {
                        return;
                    }
                    case MotorDynamics::DType::kSpeed:
                    {
                        speed = static_cast<int32_t>((*dynamics.getSpeed() / (m_wheel_radius * M_PI * 2)).to(1 / units::s) * m_ticks_per_rev);
                        m_left->setM1M2Speed(speed, speed);
                        m_right->setM1M2Speed(speed, speed);

                        return;
                    }
                    case MotorDynamics::DType::kSpeedAccel:
                    {
                        speed = static_cast<int32_t>((*dynamics.getSpeed() / (m_wheel_radius * M_PI * 2)).to(1 / units::s) * m_ticks_per_rev);
                        accel = static_cast<uint32_t>((*dynamics.getAcceleration() / (m_wheel_radius * M_PI * 2)).to(1 / (units::s * units::s)) * m_ticks_per_rev);
                        m_left->setM1M2SpeedAccel(accel, speed, speed);
                        m_right->setM1M2SpeedAccel(accel, speed, speed);

                        return;
                    }
                    case MotorDynamics::DType::kSpeedDist:
                    {
                        speed = static_cast<int32_t>((*dynamics.getSpeed() / (m_wheel_radius * M_PI * 2)).to(1 / units::s) * m_ticks_per_rev);
                        dist = static_cast<uint32_t>((*dynamics.getDistance() / (m_wheel_radius * M_PI * 2)).to(units::none) * m_ticks_per_rev);
                        m_left->setM1M2SpeedDist(speed, dist, speed, dist, respectBuffer);
                        m_right->setM1M2SpeedDist(speed, dist, speed, dist, respectBuffer);

                        return;
                    }
                    case MotorDynamics::DType::kSpeedAccelDist:
                    {
                        speed = static_cast<int32_t>((*dynamics.getSpeed() / m_wheel_radius / (units::pi * 2))() * m_ticks_per_rev);
                        dist = static_cast<uint32_t>((*dynamics.getDistance() / m_wheel_radius / (units::pi * 2))() * m_ticks_per_rev);
                        accel = static_cast<uint32_t>((*dynamics.getAcceleration() / m_wheel_radius / (units::pi * 2))() * m_ticks_per_rev);
                        m_left->setM1M2SpeedAccelDist(accel, speed, dist, speed, dist, respectBuffer);
                        m_right->setM1M2SpeedAccelDist(accel, speed, dist, speed, dist, respectBuffer);

                        return;
                    }
                    case MotorDynamics::DType::kSpeedAccelDecelDist:
                    {
                        // last test - this wasn't working
                        //setM1M2SpeedAccelDecelDist(accel, speed, decel, dist, accel, speed, decel, dist, static_cast<uint8_t>(respectBuffer));

                        speed = static_cast<int32_t>((*dynamics.getSpeed() / m_wheel_radius / (units::pi * 2))() * m_ticks_per_rev);
                        dist = static_cast<uint32_t>((*dynamics.getDistance() / m_wheel_radius / (units::pi * 2))() * m_ticks_per_rev);
                        accel = static_cast<uint32_t>((*dynamics.getAcceleration() / m_wheel_radius / (units::pi * 2))() * m_ticks_per_rev);
                        decel = static_cast<uint32_t>((*dynamics.getDeceleration() / m_wheel_radius / (units::pi * 2))() * m_ticks_per_rev);

                        // Call these separately to avoid issues from prior attempt
                        m_left->setM1SpeedAccelDecelDist(accel, speed, decel, dist, respectBuffer);
                        m_right->setM2SpeedAccelDecelDist(accel, speed, decel, dist, respectBuffer);
                        m_right->setM1SpeedAccelDecelDist(accel, speed, decel, dist, respectBuffer);
                        m_left->setM2SpeedAccelDecelDist(accel, speed, decel, dist, respectBuffer);

                        return;
                    }
                    default:
                    {
                        assert(false);
                    }
                }
            }

            std::tuple<units::Distance, units::Velocity> TwoRoboclawAppendageDrivetrain::getDistAndVel(const Motor& motor)
    		{
    			return std::tuple<units::Distance, units::Velocity>(readEncoder(motor), readEncoderVelocity(motor));
    		}

            std::tuple<units::Distance, units::Velocity> TwoRoboclawAppendageDrivetrain::getDistAndVel(bool side)
            {
                std::vector<units::Distance> d;
                std::vector<units::Velocity> v;
                std::vector<Motor> motors;
                if(side)
                {
                    motors = {Motor::kFrontRight, Motor::kBackRight};
                }
                else
                {
                    motors = {Motor::kFrontLeft, Motor::kBackLeft};
                }
                v = readEncoderVelocities(motors);
                d = readEncoders(motors);
                d[0] = (d[0] + d[1])/2;
                v[0] = (v[0] + v[1])/2;
                return std::tuple<units::Distance, units::Velocity>(d[0], v[0]);
            }

            void TwoRoboclawAppendageDrivetrain::stop()
            {
                m_left->stop();
                m_right->stop();
            }

            bool TwoRoboclawAppendageDrivetrain::diagnostic()
            {
                // todo
                return 0;
            }
        }
    }//subsystem
}//rip
