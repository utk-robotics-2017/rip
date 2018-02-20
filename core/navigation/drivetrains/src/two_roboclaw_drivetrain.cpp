#include "drivetrains/two_roboclaw_drivetrain.hpp"
#include <motor_controllers/motor_dynamics.hpp>
#include <drivetrains/exceptions.hpp>
#include <cmath>

namespace rip
{
    namespace navigation
    {
        namespace drivetrains
        {
            TwoRoboclawDrivetrain::TwoRoboclawDrivetrain(const std::string& name, std::shared_ptr<TwoRoboclawDrivetrain::Roboclaw> left, std::shared_ptr<TwoRoboclawDrivetrain::Roboclaw> right, std::shared_ptr<TwoRoboclawDrivetrain::NavX> navx)
                : Drivetrain(name)
                , m_left(left)
                , m_right(right)
                , m_navx(navx)
            {}

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

            void TwoRoboclawDrivetrain::drive(double front_left, double front_right, double back_left, double back_right)
            {
                if (std::abs(front_left) > 1 || std::abs(front_right) > 1 || std::abs(back_left) > 1 || std::abs(back_right) > 1)
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

            void TwoRoboclawDrivetrain::drive(const MotorDynamics& front_left, const MotorDynamics& front_right, const MotorDynamics& back_left, const MotorDynamics& back_right)
            {
                m_left->setDynamics(Roboclaw::Motor::kM1, front_left);
                m_right->setDynamics(Roboclaw::Motor::kM1, front_right);

                m_left->setDynamics(Roboclaw::Motor::kM2, back_left);
                m_right->setDynamics(Roboclaw::Motor::kM2, back_right);

            }

            void TwoRoboclawDrivetrain::stop()
            {
                stop(true);
            }

            bool TwoRoboclawDrivetrain::diagnostic()
            {
                // todo
            }

            void TwoRoboclawDrivetrain::stop(bool brake)
            {
                m_left->drive(0);
                m_right->drive(0);
            }
        }
    }//subsystem
}//rip
