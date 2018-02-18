#include "two_roboclaw_drivetrain.hpp"
#include <roboclaw/motor_dynamics.hpp>

namespace rip
{
    namespace subsystem
    {
        void TwoRoboclawDriveTrain::drive(double power)
        {
            m_left->drive(32767 * power);
            m_right->drive(32767 * power)
        }

        void TwoRoboclawDriveTrain::drive(double left, double right)
        {
            m_left->drive(32767 * left);
            m_right->drive(32767 * right);
        }

        void TwoRoboclawDriveTrain::drive(double front_left, double front_right, double back_left, double back_rightk)
        {
            m_left->drive(Roboclaw::Motor::kM1, 32767 * power);
            m_right->drive(Roboclaw::Motor::kM1, 32767 * power);

            m_left->drive(Roboclaw::Motor::kM2, 32767 * power);
            m_right->drive(Roboclaw::Motor::kM2, 32767 * power);
        }

        void TwoRoboclawDriveTrain::drive(const MotorDynamics& command)
        {
            m_left->setDynamics(command);
            m_right->setDynamics(command)
        }

        void TwoRoboclawDriveTrain::drive(const MotorDynamics& left, const MotorDynamics& right)
        {
            m_left->setDynamics(left);
            m_right->setDynamics(right);
        }

        void TwoRoboclawDriveTrain::drive(const MotorDynamics& front_left, const NavCommd& front_right, const MotorDynamics& back_left, const MotorDynamics& back_right)
        {
            m_left->setDynamics(Roboclaw::Motor::kM1, front_left);
            m_right->setDynamics(Roboclaw::Motor::kM1, front_right;

            m_left->setDynamics(Roboclaw::Motor::kM2, back_left);
            m_right->setDynamics(Roboclaw::Motor::kM2, back_right);

        }

        void TwoRoboclawDriveTrain::stop(bool brake = false)
        {
            m_left->drive(0);
            m_right->drive(0);
        }
    }//subsystem
}//rip
