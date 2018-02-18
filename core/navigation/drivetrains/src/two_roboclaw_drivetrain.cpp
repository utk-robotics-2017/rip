#include "two_roboclaw_drivetrain.hpp"

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

        void TwoRoboclawDriveTrain::drive(const NavCommand& command)
        {

        }

        void TwoRoboclawDriveTrain::drive(const NavCommand& left, const NavCommand& right)
        {

        }

        void TwoRoboclawDriveTrain::drive(const NavCommand& front_left, const NavCommand& front_right, const NavCommand& back_left, const NavCommand& back_right)
        {

        }

        void TwoRoboclawDriveTrain::stop(bool brake = false)
        {

        }
    }//subsystem
}//rip
