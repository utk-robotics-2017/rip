#ifndef TURN_BY_ANGLE_HPP
#define TURN_BY_ANGLE_HPP

#include <json.hpp>
#include <drivetrains/drivetrain.hpp>
#include <chrono>
#include <framework/timeout_action.hpp>
#include <navx/navx.hpp>
#include <pid/pid_output.hpp>
#include <pid/pid.hpp>
#include <cmath>

#include "navigation_actions/turn_to_angle.hpp"

namespace rip
{
    namespace navigation
    {
        namespace actions
        {
            class TurnByAngle : public TurnToAngle
            {
                using Drivetrain = drivetrains::Drivetrain;
                using NavX = navx::NavX;
            public:
                TurnByAngle(const std::string& name,
                            std::shared_ptr<Drivetrain> drivetrain,
                            std::shared_ptr<NavX> navx, const nlohmann::json& config);

                /**
                 * Run once before the main code
                 */
                virtual void setup(nlohmann::json& state) override;
            };
        }
    }
}

#endif // TURN_TO_ANGLE_HPP
