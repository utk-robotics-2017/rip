#include "navigation_actions/drive_straight.hpp"

namespace rip
{
    namespace navigation
    {
        namespace actions
        {
            DriveStraight::DriveStraight(std::shared_ptr<drivetrains::DriveTrain> drivetrain, const units::Distance& distance, double p, double i, double d)
                : m_use_time(false)
                , m_distance(distance)
                , m_drivetrain(drivetrain)
            {}

            DriveStraight::DriveStraight(std::shared_ptr<drivetrains::DriveTrain> drivetrain, const units::Time& time)
                : m_use_time(true)
                , m_time(time)
                , m_drivetrain(drivetrain)
            {}

            bool DriveStraight::isFinished()
            {
                // todo
            }

            void DriveStraight::update(nlohmann::json& state)
            {
                // todo
            }

            void DriveStraight::setup(nlohmann::json& state)
            {
                // todo
            }

            void DriveStraight::teardown(nlohmann::json& state)
            {
                // todo
            }
        }
    }
}
