#ifndef TURN_TO_ANGLE_HPP
#define TURN_TO_ANGLE_HPP

#include <json.hpp>
#include <drivetrains/drivetrain.hpp>
#include <chrono>
#include <framework/action.hpp>
#include <navx/navx.hpp>
#include <pid/pid_output.hpp>
#include <pid/pid.hpp>
#include <cmath>

namespace rip
{
    namespace navigation
    {
        namespace actions
        {
            class TurnToAngle : public framework::Action, public pid::PidOutput
            {
                using Drivetrain = drivetrains::Drivetrain;
                using NavX = navx::NavX;

            public:
                /**
                 * turns to angle, relative to current angle, by driving
                 * wheels in opposite directions at the same speed.
                 * @param name       name of action
                 * @param drivetrain drive train
                 * @param navx       NavX (Imu)
                 * @oaram config     Parameters used for turning to angle
                 */
                TurnToAngle(const std::string& name,
                            std::shared_ptr<Drivetrain> drivetrain,
                            std::shared_ptr<NavX> navx, const nlohmann::json& config);

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

                virtual void set(double output);

            private:
                std::shared_ptr<Drivetrain> m_drivetrain;
                std::shared_ptr<NavX> m_navx;

                bool m_first;
                units::Angle m_start_angle;
                units::Angle m_turn_angle;
                units::Angle m_setpoint;
                units::Acceleration m_max_accel;

                /* PID Control for rotational correction */
                std::unique_ptr<pid::PidController> m_pid;

            };
        }
    }
}

#endif // TURN_TO_ANGLE_HPP
