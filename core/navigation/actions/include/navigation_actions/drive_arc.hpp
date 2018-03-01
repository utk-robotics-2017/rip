#ifndef DRIVE_ARC_HPP
#define DRIVE_ARC_HPP

#include <json.hpp>
#include <misc/logger.hpp>
#include <fmt/format.h>
#include <framework/action.hpp>
#include <drivetrains/drivetrain.hpp>

namespace rip
{
    namespace navigation
    {
        namespace actions
        {
            class DriveArc : public framework::Action
            {
            public:
                using Motor = rip::navigation::drivetrains::Drivetrain::Motor;
            
                /**
                 * Drives the robot in an arc with specified radius, angular distance,
                 * angular velocity for the turn
                 * @param name       Action name
                 * @param direction  0 means left turn. 1 means right turn
                 * @param drivetrain pointer to differential drivetrain.
                 * @param speed      angular velocity for the turn
                 * @param angle      angle to tunr
                 * @param radius     radius size of arc turn
                 * @param axleLength width between left and right wheels.
                 */
                DriveArc(const std::string& name, bool direction,
                    std::shared_ptr<drivetrains::Drivetrain> drivetrain,
                    const units::AngularVelocity& speed, const units::Angle& angle,
                    const units::Distance& radius, const units::Distance& axleLength);

                /**
                 * Drives the robot in an arc with specified radius, arc length,
                 * angular velocity for the turn
                 * @param name       Action name
                 * @param direction  0 means left turn. 1 means right turn
                 * @param drivetrain pointer to differential drivetrain.
                 * @param speed      angular velocity for the turn
                 * @param arcLength  Linear distance that for the robot to travel
                 * @param radius     radius size of arc turn
                 * @param axleLength width between left and right wheels.
                 */
                DriveArc(const std::string& name, bool direction,
                    std::shared_ptr<drivetrains::Drivetrain> drivetrain,
                    const units::Velocity& speed, const units::Distance& arcLength,
                    const units::Distance& radius, const units::Distance& axleLength);
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
                 * //TODO: support for multiple drivetrain types
                 * Returns the average distance read from all four wheels
                 * @return average distance
                 */
                units::Distance readAverageDistance();
                /**
                 * Run once after finished
                 */
                virtual void teardown(nlohmann::json& state) override;

            private:
                bool m_direction;
                std::shared_ptr<drivetrains::Drivetrain> m_drivetrain;
                units::AngularVelocity m_angularSpeed;
                units::Velocity m_linearSpeed;
                units::Angle m_angle;
                /**
                 * radius is distance from center of robot/differential drivetrain
                 * to center of circle of arc (ICC)
                 */
                units::Distance m_arcLength, m_radius, m_axleLength, m_traveled, m_init;
            };
        }
    }
}

#endif // DRIVE_ARC_HPP
