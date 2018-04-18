#ifndef DRIVE_STRAIGHT_UNTIL_OTHER_HPP
#define DRIVE_STRAIGHT_UNTIL_OTHER_HPP

#include <navigation_actions/drive_straight.hpp>
#include <framework/condition.hpp>

namespace rip
{
    namespace navigation
    {
        namespace actions
        {
            class DriveStraightUntil : public rip::navigation::actions::DriveStraight
            {
            protected:
                using Condition = rip::framework::Condition;
            public:
                /**
                * Constructor
                */
                DriveStraightUntil(const std::string& name,
                                   std::shared_ptr<Drivetrain> drivetrain,
                                   std::shared_ptr<Imu> imu,
                                   std::shared_ptr<Condition> condition,
                                   const nlohmann::json& config);

                virtual bool isFinished() override;

            protected:
                std::shared_ptr<Condition> m_condition;
            };
        }
    }
}

#endif // DRIVE_STRAIGHT_UNTIL_OTHER_HPP
