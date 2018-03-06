#ifndef DRIVE_ARC_HPP
#define DRIVE_ARC_HPP

#include <json.hpp>
#include <misc/logger.hpp>
#include <fmt/format.h>
#include <framework/action.hpp>
#include <drivetrains/drivetrain.hpp>
#include <navx/navx.hpp>
#include "exceptions.hpp"

namespace rip
{
    namespace navigation
    {
        namespace actions
        {
            using NavX = navx::NavX;
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
                 * @param axle_length width between left and right wheels.
                 */
                DriveArc(const std::string& name, bool direction,
                    std::shared_ptr<drivetrains::Drivetrain> drivetrain,
                    const units::AngularVelocity& speed, const units::Angle& angle,
                    const units::Distance& radius, const units::Distance& axle_length);

                /**
                 * Drives the robot in an arc with specified radius, arc length,
                 * angular velocity for the turn
                 * @param name       Action name
                 * @param direction  0 means left turn. 1 means right turn
                 * @param drivetrain pointer to differential drivetrain.
                 * @param speed      angular velocity for the turn
                 * @param arc_length  Linear distance that for the robot to travel
                 * @param radius     radius size of arc turn
                 * @param axle_length width between left and right wheels.
                 */
                DriveArc(const std::string& name, bool direction,
                    std::shared_ptr<drivetrains::Drivetrain> drivetrain,
                    const units::Velocity& speed, const units::Distance& arc_length,
                    const units::Distance& radius, const units::Distance& axle_length);
                /**
                 * drivearc. but with a navx.
                 * @param name        action name
                 * @param direction   direction to turn
                 * @param drivetrain  drivetrain
                 * @param speed       linear speed (units)
                 * @param arc_length  linear distance to drive
                 * @param radius      radius to turn around
                 * @param axle_length length of axle base
                 * @param navx        ptr to navx
                 */
                DriveArc(const std::string& name, bool direction,
                    std::shared_ptr<drivetrains::Drivetrain> drivetrain,
                    const units::Velocity& speed, const units::Distance& arc_length,
                    const units::Distance& radius, const units::Distance& axle_length,
                    std::shared_ptr<NavX> navx);

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
                units::AngularVelocity m_angular_speed;
                units::Velocity m_linear_speed;
                units::Angle m_angle;
                std::shared_ptr<NavX> m_navx;
                /**
                 * radius is distance from center of robot/differential drivetrain
                 * to center of circle of arc (ICC)
                 */
                units::Distance m_arc_length, m_radius, m_axle_length, m_traveled, m_init;
            };
        }
    }
}

#endif // DRIVE_ARC_HPP
