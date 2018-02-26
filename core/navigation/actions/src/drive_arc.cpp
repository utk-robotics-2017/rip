#include "navigation_actions/drive_arc.hpp"

namespace rip
{
    namespace navigation
    {
        namespace actions
        {
            DriveArc::DriveArc(const std::string& name, bool direction,
                std::shared_ptr<drivetrains::Drivetrain> drivetrain,
                const units::AngularVelocity& speed, const units::Angle& angle,
                units::Distance& radius, units::Distance& axleLength)
                : Action(name)
                , m_direction(direction)
                , m_drivetrain(drivetrain)
                , m_angularSpeed(speed)
                , m_angle(angle)
                , m_radius(radius)
                , m_axleLength(axleLength)
            {
                m_arcLength = angle.to(units::rad) * radius;
                m_linearSpeed = radius * speed / units::rad;
            }

            DriveArc::DriveArc(const std::string& name, bool direction,
                std::shared_ptr<drivetrains::Drivetrain> drivetrain,
                const units::Velocity& speed, const units::Distance& arcLength,
                units::Distance& radius, units::Distance& axleLength)
                : Action(name)
                , m_direction(direction)
                , m_drivetrain(drivetrain)
                , m_linearSpeed(speed)
                , m_arcLength(arcLength)
                , m_radius(radius)
                , m_axleLength(axleLength)
            {
                m_angle = arcLength / radius * units::rad;
                m_angularSpeed = speed / radius * units::rad;
            }

            bool DriveArc::isFinished()
            {
                m_traveled = readAverageDistance();
                return m_arcLength >= (m_traveled - m_init);
            }

            void DriveArc::update(nlohmann::json& state)
            {
                // todo
                return;
            }

            void DriveArc::setup(nlohmann::json& state)
            {
                std::vector<units::Distance> dist;
                units::Velocity v1, v2;
                m_init = readAverageDistance();

                misc::Logger::getInstance()->debug(fmt::format("initial(offset) distance(ft): {}", m_init.to(units::ft)));
                motorcontrollers::MotorDynamics dynamicsLeft, dynamicsRight;

                misc::Logger::getInstance()->debug(fmt::format("arc turn intended angular velocity (rev/min): {}"
                , m_angularSpeed.to(units::rev / units::minute)));

                misc::Logger::getInstance()->debug(fmt::format("arc turn intended linear velocity(in/s): {}"
                , m_linearSpeed.to(units::in / units::s)));

                v1 = m_angularSpeed * (m_radius + m_axleLength/2 ) / units::rad;
                v2 = m_angularSpeed * (m_radius - m_axleLength/2 ) / units::rad;
                if(!m_direction) // left turn
                {
                    dynamicsLeft.setSpeed(v2);
                    dynamicsRight.setSpeed(v1);
                }
                else //right turn
                {
                    dynamicsLeft.setSpeed(v1);
                    dynamicsRight.setSpeed(v2);
                }
                m_drivetrain->drive(dynamicsLeft, dynamicsRight);
            }

            units::Distance DriveArc::readAverageDistance()
            {
                units::Distance sum=0;
                std::vector<units::Distance> dist = m_drivetrain->readEncoders({Motor::kFrontLeft,
                    Motor::kFrontLeft, Motor::kFrontRight, Motor::kBackLeft, Motor::kBackRight});
                for(int i=0; i<static_cast<int>(dist.size()); i++)
                {
                    sum += dist[i];
                }
                sum /= dist.size();
                return sum;
            }

            void DriveArc::teardown(nlohmann::json& state)
            {
                // todo
                m_drivetrain->stop();
            }
        }
    }
}
