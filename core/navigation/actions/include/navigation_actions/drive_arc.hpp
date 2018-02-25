#ifndef DRIVE_ARC_HPP
#define DRIVE_ARC_HPP

#include <json.hpp>

#include <framework/action.hpp>

namespace rip
{
    namespace navigation
    {
        namespace actions
        {
            class DriveArc : public framework::Action
            {
            public:

                /**
                 * [DriveArc description]
                 * @param name       Action name
                 * @param direction  0 means left turn. 1 means right turn
                 * @param drivetrain pointer to differential drivetrain.
                 * @param speed      angular velocity for the turn
                 * @param angle      between
                 * @param radius     size of arc
                 * @param axleLength width between left and right wheels.
                 */
                DriveArc(const std::string& name, bool direction,
                    std::shared_ptr<drivetrains::Drivetrain> drivetrain,
                    const units::AngularVelocity& speed, const units::Angle& angle,
                    units::Distance& radius, units::Distance& axleLength);

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
                units::Angle m_desiredAngle;
                std::shared_ptr<drivetrains::Drivetrain> m_drivetrain;
                /**
                 * radius is distance from center of robot/differential drivetrain
                 * to center of circle of arc (ICC)
                 */
                units::Distance m_axleLength, m_radius, m_arcLength;
                units::AngularVelocity m_speed;
                bool m_direction;
            };
        }
    }
}

#endif // DRIVE_ARC_HPP
