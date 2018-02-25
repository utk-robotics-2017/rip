#ifndef TURN_TO_ANGLE_HPP
#define TURN_TO_ANGLE_HPP

#include <json.hpp>
#include <drivetrains/drivetrain.hpp>
#include <chrono>
#include <framework/action.hpp>
#include <navx/navx.hpp>
#include <cmath>

namespace rip
{
    namespace navigation
    {
        namespace actions
        {
            //currently depends on differential drivetrain
            using NavX = navx::NavX;
            class TurnToAngle : public framework::Action
            {
            public:
                /**
                 * turns to angle, relative to current angle, by driving
                 * wheels in opposite directions at the same speed.
                 * @param name       name of action
                 * @param drivetrain drive train
                 * @param speed      angular velocity of turn
                 * @param angle      how many degrees t
                 * @param navx       pointer to navx
                 * @param radius     distance between wheel and the center of
                 *  the robot (top down)
                 */
                TurnToAngle(const std::string& name,
                    std::shared_ptr<drivetrains::Drivetrain> drivetrain,
                    const units::AngularVelocity& speed, const units::Angle& angle,
                    std::shared_ptr<NavX> navx, units::Distance& radius);
                /**
                * Returns whether or not the action has finished execution.
                * Returns true when navX reports change in angle that was
                * requested
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
                units::AngularVelocity m_speed;
                std::shared_ptr<drivetrains::Drivetrain> m_drivetrain;
                std::shared_ptr<NavX> m_navx;
                units::Angle m_desiredAngle;
                units::Distance m_c2wRadius;
            };
        }
    }
}

#endif // TURN_TO_ANGLE_HPP
