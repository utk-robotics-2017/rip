#include "drivetrains/two_roboclaw_drivetrain.hpp"
#include <motor_controllers/motor_dynamics.hpp>
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
            TwoRoboclawDrivetrain::TwoRoboclawDrivetrain(const std::string& name,
                std::shared_ptr<TwoRoboclawDrivetrain::Roboclaw> left,
                std::shared_ptr<TwoRoboclawDrivetrain::Roboclaw> right,
                std::shared_ptr<TwoRoboclawDrivetrain::NavX> navx)
                : Drivetrain(name)
                , m_left(left)
                , m_right(right)
                , m_navx(navx)
            {}

            TwoRoboclawDrivetrain::~TwoRoboclawDrivetrain()
            {
                stop();
            }

            void TwoRoboclawDrivetrain::drive(double power)
            {
                if (std::abs(power) > 1)
                {
                    throw OutOfRangeException("out of range");
                }
                m_left->drive(32767 * power);
                m_right->drive(32767 * power);
            }

            void TwoRoboclawDrivetrain::drive(double left, double right)
            {
                if (std::abs(left) > 1 || std::abs(right) > 1)
                {
                    throw OutOfRangeException("out of range");
                }
                m_left->drive(32767 * left);
                m_right->drive(32767 * right);
            }

            void TwoRoboclawDrivetrain::drive(double front_left, double front_right, double back_left,
                double back_right)
            {
                if (std::abs(front_left) > 1 || std::abs(front_right) > 1 || std::abs(back_left) > 1 ||
                std::abs(back_right) > 1)
                {
                    throw OutOfRangeException("out of range");
                }

                m_left->drive(Roboclaw::Motor::kM1, 32767 * front_left);
                m_right->drive(Roboclaw::Motor::kM1, 32767 * front_right);

                m_left->drive(Roboclaw::Motor::kM2, 32767 * back_left);
                m_right->drive(Roboclaw::Motor::kM2, 32767 * back_right);
            }

            void TwoRoboclawDrivetrain::drive(const MotorDynamics& command)
            {
                m_left->setDynamics(command);
                m_right->setDynamics(command);
            }

            void TwoRoboclawDrivetrain::drive(const MotorDynamics& left, const MotorDynamics& right)
            {
                m_left->setDynamics(left);
                m_right->setDynamics(right);
            }

            void TwoRoboclawDrivetrain::drive(const MotorDynamics& front_left, const MotorDynamics& front_right,
                 const MotorDynamics& back_left, const MotorDynamics& back_right)
            {
                m_left->setDynamics(Roboclaw::Motor::kM1, front_left);
                m_right->setDynamics(Roboclaw::Motor::kM1, front_right);

                m_left->setDynamics(Roboclaw::Motor::kM2, back_left);
                m_right->setDynamics(Roboclaw::Motor::kM2, back_right);

            }

            std::vector<units::Distance> TwoRoboclawDrivetrain::readEncoders(const std::vector<Motor>& motors)
            {
                std::vector<units::Distance> data;
                for(uint i=0; i<motors.size(); i++)
                {

                    switch(motors[i])
                    {

                        case Motor::kFrontLeft:
                        {
                            data.push_back(m_left->readEncoder(Roboclaw::Motor::kM1));
                            break;
                        }
                        case Motor::kFrontRight:
                        {
                            data.push_back(m_right->readEncoder(Roboclaw::Motor::kM1));
                            break;
                        }
                        case Motor::kBackLeft:
                        {
                            data.push_back(m_left->readEncoder(Roboclaw::Motor::kM2));
                            break;
                        }
                        case Motor::kBackRight:
                        {
                            data.push_back(m_right->readEncoder(Roboclaw::Motor::kM2));
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

            units::Distance TwoRoboclawDrivetrain::readEncoder(const Motor& motor)
            {
                switch(motor)
                {
                    case Motor::kFrontLeft:
                    {
                        return m_left->readEncoder(Roboclaw::Motor::kM1);
                    }
                    case Motor::kFrontRight:
                    {
                        return m_right->readEncoder(Roboclaw::Motor::kM1);
                    }
                    case Motor::kBackLeft:
                    {
                        return m_left->readEncoder(Roboclaw::Motor::kM2);
                    }
                    case Motor::kBackRight:
                    {
                        return m_right->readEncoder(Roboclaw::Motor::kM2);
                    }
                    default:
                    {
                        throw InvalidMotorException("Invalid motor");
                    }
                }
            }

            std::vector<units::Velocity> TwoRoboclawDrivetrain::readEncoderVelocities(
                const std::vector<Motor>& motors)
            {
                std::vector<units::Velocity> data;
                for(uint i=0; i<motors.size(); i++)
                {
                    switch(motors[i])
                    {
                        case Motor::kFrontLeft:
                        {
                            data.push_back(m_left->readEncoderVelocity(Roboclaw::Motor::kM1));
                        }
                        case Motor::kFrontRight:
                        {
                            data.push_back(m_right->readEncoderVelocity(Roboclaw::Motor::kM1));
                        }
                        case Motor::kBackLeft:
                        {
                            data.push_back(m_left->readEncoderVelocity(Roboclaw::Motor::kM2));
                        }
                        case Motor::kBackRight:
                        {
                            data.push_back(m_right->readEncoderVelocity(Roboclaw::Motor::kM2));
                        }
                        default:
                        {
                            throw InvalidMotorException(fmt::format("Invalid motor, parameter {}", i+1));
                        }
                    }
                }
                return data;
            }

            units::Velocity TwoRoboclawDrivetrain::readEncoderVelocity(const Motor& motor)
            {
                switch(motor)
                {
                    case Motor::kFrontLeft:
                    {
                        return m_left->readEncoderVelocity(Roboclaw::Motor::kM1);
                    }
                    case Motor::kFrontRight:
                    {
                        return m_right->readEncoderVelocity(Roboclaw::Motor::kM1);
                    }
                    case Motor::kBackLeft:
                    {
                        return m_left->readEncoderVelocity(Roboclaw::Motor::kM2);
                    }
                    case Motor::kBackRight:
                    {
                        return m_right->readEncoderVelocity(Roboclaw::Motor::kM2);
                    }
                    default:
                    {
                        throw InvalidMotorException("Invalid motor");
                    }
                }
            }

            units::Angle TwoRoboclawDrivetrain::readGyro()
            {
                return m_navx->getAngle();
            }

            void TwoRoboclawDrivetrain::resetEncoders()
            {
                m_left->resetEncoders();
                m_right->resetEncoders();
            }

            void TwoRoboclawDrivetrain::stop()
            {
                m_left->drive(0);
                m_right->drive(0);
            }

            bool TwoRoboclawDrivetrain::diagnostic()
            {
                // todo
                return 0;
            }
        }
    }//subsystem
}//rip
