#include "drivetrains/one_roboclaw_drivetrain.hpp"
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
            TwoRoboclawDrivetrain::TwoRoboclawDrivetrain(const std::string& name,
                std::shared_ptr<TwoRoboclawDrivetrain::Roboclaw> rclaw,
                std::shared_ptr<TwoRoboclawDrivetrain::NavX> navx)
                : Drivetrain(name)
                , m_rclaw(rclaw)
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
                m_rclaw->drive(32767 * power);
            }

            void TwoRoboclawDrivetrain::drive(double left, double right)
            {
                if (std::abs(left) > 1 || std::abs(right) > 1)
                {
                    throw OutOfRangeException("out of range");
                }
                m_rclaw->drive(Roboclaw::Motor::kM1, 32767 * left);
                m_rclaw->drive(Roboclaw::Motor::kM2, 32767 * right);
            }

            void TwoRoboclawDrivetrain::drive(const MotorDynamics& command)
            {
                m_rclaw->setDynamics(command);
            }

            void TwoRoboclawDrivetrain::drive(const MotorDynamics& left, const MotorDynamics& right)
            {
                m_rclaw->setDynamics(Robclaw::Motor::kM1, left);
                m_rclaw->setDynamics(Robclaw::Motor::kM2, right);
            }

            void TwoRoboclawDrivetrain::stop()
            {
                m_rclaw->drive(0);
            }

            bool TwoRoboclawDrivetrain::diagnostic()
            {
                // todo
                return 0;
            }
        }
    }
}
