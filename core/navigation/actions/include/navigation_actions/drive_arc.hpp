#ifndef DRIVE_ARC_HPP
#define DRIVE_ARC_HPP

#include <json.hpp>
#include <misc/logger.hpp>
#include <fmt/format.h>
#include <framework/action.hpp>
#include <drivetrains/drivetrain.hpp>
//#include <navx/navx.hpp>
#include "exceptions.hpp"
#include <chrono>


namespace rip
{
    namespace navigation
    {
        namespace actions
        {
            using Motor = rip::navigation::drivetrains::Drivetrain::Motor;
            // using NavX = navx::NavX;
            class DriveArc : public framework::Action
            {
            public:

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
                DriveArc(const std::string& name, bool right,
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
                DriveArc(const std::string& name, bool right,
                    std::shared_ptr<drivetrains::Drivetrain> drivetrain,
                    const units::Velocity& speed, const units::Distance& arc_length,
                    const units::Distance& radius, const units::Distance& axle_length);                /**
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
                bool m_right;
                std::shared_ptr<drivetrains::Drivetrain> m_drivetrain;
                units::AngularVelocity m_angular_speed;
                units::Velocity m_linear_speed;
                units::Angle m_angle;
                //std::shared_ptr<NavX> m_navx;
                const double k_dead_zone = 0.0001; //percent
                /**
                 * radius is distance from center of robot/differential drivetrain
                 * to center of circle of arc (ICC)
                 */
                units::Distance m_arc_length, m_radius, m_axle_length, m_traveled, m_init_dist;
                units::Angle m_init_angle;
                std::chrono::time_point<std::chrono::system_clock> m_start_time;
                units::Velocity m_velocity_inner, m_velocity_outer;
                std::array<units::Velocity, 2> m_adjusted_speeds;

            };
        }
    }
}

#endif // DRIVE_ARC_HPP
