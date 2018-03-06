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
                , m_roboclaw(rclaw)
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
                m_roboclaw->drive(32767 * power);
            }

            void OneRoboclawDrivetrain::drive(double left, double right)
            {
                if (std::abs(left) > 1 || std::abs(right) > 1)
                {
                    throw OutOfRangeException("out of range");
                }
                m_roboclaw->drive(Roboclaw::Motor::kM1, 32767 * left);
                m_roboclaw->drive(Roboclaw::Motor::kM2, 32767 * right);
            }

            void OneRoboclawDrivetrain::drive(const MotorDynamics& command)
            {
                m_roboclaw->setDynamics(command);
            }

            void OneRoboclawDrivetrain::drive(const MotorDynamics& left, const MotorDynamics& right)
            {
                m_roboclaw->setDynamics(Roboclaw::Motor::kM1, left);
                m_roboclaw->setDynamics(Roboclaw::Motor::kM2, right);
            }

            bool OneRoboclawDrivetrain::hasGyro() const
            {
                return m_navx != nullptr;
            }

            units::Angle OneRoboclawDrivetrain::readYaw() const
            {
                return m_navx->getYaw();
            }

            void OneRoboclawDrivetrain::stop()
            {
                m_roboclaw->drive(0);
            }

            bool OneRoboclawDrivetrain::diagnostic()
            {
                // todo
                return 0;
            }
        }
    }
}
