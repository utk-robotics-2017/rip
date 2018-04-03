#include "drivetrains/two_roboclaw_appendage_drivetrain.hpp"
#include <drivetrains/exceptions.hpp>
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
                std::shared_ptr<TwoRoboclawAppendageDrivetrain::Roboclaw> left,
                std::shared_ptr<TwoRoboclawAppendageDrivetrain::Roboclaw> right,
                std::shared_ptr<TwoRoboclawAppendageDrivetrain::NavX> navx)
                : Drivetrain(name)
                , m_left(left)
                , m_right(right)
                , m_navx(navx)
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
                m_left->setDynamics(command);
                m_right->setDynamics(command);
            }

            void TwoRoboclawAppendageDrivetrain::drive(const MotorDynamics& left, const MotorDynamics& right)
            {
                m_left->setDynamics(left);
                m_right->setDynamics(right);
            }

            void TwoRoboclawAppendageDrivetrain::drive(const MotorDynamics& front_left, const MotorDynamics& front_right,
                 const MotorDynamics& back_left, const MotorDynamics& back_right)
            {
                m_left->setDynamics(0, front_left);
                m_right->setDynamics(0, front_right);

                m_left->setDynamics(1, back_left);
                m_right->setDynamics(1, back_right);

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
                            data.push_back(m_left->readEncoder(0));
                            break;
                        }
                        case Motor::kFrontRight:
                        {
                            data.push_back(m_right->readEncoder(0));
                            break;
                        }
                        case Motor::kBackLeft:
                        {
                            data.push_back(m_left->readEncoder(1));
                            break;
                        }
                        case Motor::kBackRight:
                        {
                            data.push_back(m_right->readEncoder(1));
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
                        return m_left->readEncoder(0);
                    }
                    case Motor::kFrontRight:
                    {
                        return m_right->readEncoder(0);
                    }
                    case Motor::kBackLeft:
                    {
                        return m_left->readEncoder(1);
                    }
                    case Motor::kBackRight:
                    {
                        return m_right->readEncoder(1);
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
                            data.push_back(m_left->readEncoderSpeed(0));
                        }
                        case Motor::kFrontRight:
                        {
                            data.push_back(m_right->readEncoderSpeed(0));
                        }
                        case Motor::kBackLeft:
                        {
                            data.push_back(m_left->readEncoderSpeed(1));
                        }
                        case Motor::kBackRight:
                        {
                            data.push_back(m_right->readEncoderSpeed(1));
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
                        return m_left->readEncoderSpeed(0);
                    }
                    case Motor::kFrontRight:
                    {
                        return m_right->readEncoderSpeed(0);
                    }
                    case Motor::kBackLeft:
                    {
                        return m_left->readEncoderSpeed(1);
                    }
                    case Motor::kBackRight:
                    {
                        return m_right->readEncoderSpeed(1);
                    }
                    default:
                    {
                        throw InvalidMotorException("Invalid motor");
                    }
                }
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
