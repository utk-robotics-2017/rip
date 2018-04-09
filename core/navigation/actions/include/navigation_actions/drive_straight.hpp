#ifndef DRIVE_STRAIGHT_HPP
#define DRIVE_STRAIGHT_HPP

#include <json.hpp>

#include <framework/action.hpp>
#include <drivetrains/drivetrain.hpp>
#include <pid/pid_output.hpp>
#include <pid/pid.hpp>
#include <navx/navx.hpp>

#include <chrono>


namespace rip
{
    namespace navigation
    {
        namespace actions
        {
            class DriveStraight : public framework::Action, public pid::PidOutput
            {
            public:
                DriveStraight(const std::string& name, std::shared_ptr<drivetrains::Drivetrain> drivetrain,
                     std::shared_ptr<navx::NavX> navx, const units::Distance& distance, const units::Velocity& speed, units::Acceleration max_accel, double p, double i, double d);

                DriveStraight(const std::string& name, std::shared_ptr<drivetrains::Drivetrain> drivetrain,
                     std::shared_ptr<navx::NavX> navx, const units::Time& time, const units::Velocity& speed, units::Acceleration max_accel, double p, double i, double d);

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


                virtual void set(double output);

            private:
                bool m_use_time;

                /* distance to go */
                units::Distance m_distance;
                units::Distance m_init_encoder;

                units::Time m_time;
                std::chrono::time_point<std::chrono::system_clock> m_start_time;
                std::chrono::time_point<std::chrono::system_clock> m_last_time;

                /* Target speed for driving straight */
                units::Velocity m_speed;

                /* Maximum acceleration/deceleration */
                units::Acceleration m_max_accel;

                /* Drivetrain subsystem -- used to abstract the control of the drive motors */
                std::shared_ptr<drivetrains::Drivetrain> m_drivetrain;

                /* NavX IMU -- used for gyro & accelerometer */
                std::shared_ptr<navx::NavX> m_navx;

                /* PID Control for rotational correction */
                std::unique_ptr<pid::PidController> m_pid;

                /* Initial yaw -- should try to maintain the same yaw since we are driving straight */
                units::Angle m_initial_yaw;

                bool m_finished;
            };

        }
    }
}

#endif // DRIVE_STRAIGHT_HPP
