#include "drivetrains/one_roboclaw_drivetrain.hpp"
#include <motor_controllers/motor_dynamics.hpp>
#include <drivetrains/exceptions.hpp>
#include <cmath>

namespace rip
{
    namespace navigation
    {
        namespace drivetrains
        {
            OneRoboclawDrivetrain::OneRoboclawDrivetrain(const std::string& name,
                std::shared_ptr<OneRoboclawDrivetrain::Roboclaw> rclaw,
                std::shared_ptr<OneRoboclawDrivetrain::NavX> navx)
                : Drivetrain(name)
                , m_rclaw(rclaw)
                , m_navx(navx)
            {}

            OneRoboclawDrivetrain::~OneRoboclawDrivetrain()
            {
                stop();
            }

            void OneRoboclawDrivetrain::drive(double power)
            {
                if (std::abs(power) > 1)
                {
                    throw OutOfRangeException("out of range");
                }
                m_rclaw->drive(32767 * power);
            }

            void OneRoboclawDrivetrain::drive(double left, double right)
            {
                if (std::abs(left) > 1 || std::abs(right) > 1)
                {
                    throw OutOfRangeException("out of range");
                }
                m_rclaw->drive(Roboclaw::Motor::kM1, 32767 * left);
                m_rclaw->drive(Roboclaw::Motor::kM2, 32767 * right);
            }

            void OneRoboclawDrivetrain::drive(const MotorDynamics& command)
            {
                m_rclaw->setDynamics(command);
            }

            void OneRoboclawDrivetrain::drive(const MotorDynamics& left, const MotorDynamics& right)
            {
                m_rclaw->setDynamics(Roboclaw::Motor::kM1, left);
                m_rclaw->setDynamics(Roboclaw::Motor::kM2, right);
            }

            units::Angle OneRoboclawDrivetrain::readGyro()
            {
                return m_navx->getAngle();
            }

            void OneRoboclawDrivetrain::resetEncoders()
            {
                m_rclaw->resetEncoders();
            }

            void OneRoboclawDrivetrain::stop()
            {
                m_rclaw->drive(0);
            }

            bool OneRoboclawDrivetrain::diagnostic()
            {
                // todo
                return 0;
            }
        }
    }
}
