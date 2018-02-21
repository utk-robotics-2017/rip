#ifndef DRIVE_STRAIGHT_HPP
#define DRIVE_STRAIGHT_HPP

#include <json.hpp>

#include <framework/action.hpp>
#include <drivetrains/drivetrain.hpp>

namespace rip
{
    namespace navigation
    {
        namespace actions
        {
            class DriveStraight : public framework::Action
            {
            public:
                DriveStraight(std::shared_ptr<drivetrains::Drivetrain> drivetrain, const units::Distance& distance, double p, double i, double d);

                DriveStraight(std::shared_ptr<drivetrains::Drivetrain> drivetrain, const units::Time& time, const units::Velocity& speed);

                /**
                * Returns whether or not the action has finished execution.
                */
                virtual bool isFinished() override;

                /**
                 * Iteratively called until {@see Action#isFinished()} returns true
                 */
                virtual void update(nlohmann::json& state) override;

                /**
                 * Run once before the main code
                 */
                virtual void setup(nlohmann::json& state) override;

                /**
                 * Run once after finished
                 */
                virtual void teardown(nlohmann::json& state) override;

            private:
                bool m_use_time;
                units::Distance m_distance;
                units::Time m_time;
                units::Time m_start_time;
                units::Velocity m_speed;
                std::shared_ptr<drivetrains::Drivetrain> m_drivetrain;
            };

        }
    }
}

#endif // DRIVE_STRAIGHT_HPP
