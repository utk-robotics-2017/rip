#include "navigation_actions/drive_straight_until.hpp"

namespace rip
{
    namespace navigation
    {
        namespace actions
        {
            DriveStraightUntil::DriveStraightUntil(const std::string& name,
                                                   std::shared_ptr<Drivetrain> drivetrain,
                                                   std::shared_ptr<NavX> navx,
                                                   std::shared_ptr<Condition> condition,
                                                   const nlohmann::json& config)
                : DriveStraight(name, drivetrain, navx, config)
                , m_condition(condition)
            {}

            bool DriveStraightUntil::isFinished()
            {
                return m_condition->isTrue() || DriveStraight::isFinished();
            }
        }
    }
}
