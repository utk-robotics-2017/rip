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
                m_arcLength = angle * radius;
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
                m_angle = arcLength / radius;
            }

            bool DriveArc::isFinished()
            {
                return m_arcLength >=
            }

            void DriveArc::update(nlohmann::json& state)
            {
                // todo
                return;
            }

            void DriveArc::setup(nlohmann::json& state)
            {
                m_traveled = m_drivetrain
            }

            void DriveArc::teardown(nlohmann::json& state)
            {
                // todo
            }
        }
    }
}
