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
                , m_speed(speed)
                , m_angle(angle)
                , m_radius(radius)
                , m_axleLength(axleLength)
            {
                m_arcLength = angle.to(units::rad) * radius;
            }

            DriveArc::DriveArc(const std::string& name, bool direction,
                std::shared_ptr<drivetrains::Drivetrain> drivetrain,
                const units::AngularVelocity& speed, const units::Distance& arcLength,
                units::Distance& radius, units::Distance& axleLength)
                : Action(name)
                , m_direction(direction)
                , m_drivetrain(drivetrain)
                , m_speed(speed)
                , m_arcLength(arcLength)
                , m_radius(radius)
                , m_axleLength(axleLength)
            {
                m_angle = arcLength / radius * units::rad;
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
                m_init = readAverageDistance();
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
