#ifndef DRIVE_STRAIGHT_HPP
#define DRIVE_STRAIGHT_HPP

#include <json.hpp>

#include <framework/timeout_action.hpp>
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
            class DriveStraight : public framework::TimeoutAction, public pid::PidOutput
            {
            protected:
                using Drivetrain = drivetrains::Drivetrain;
                using NavX = navx::NavX;
            public:
                DriveStraight(const std::string& name, std::shared_ptr<Drivetrain> drivetrain,
                              std::shared_ptr<NavX> navx, const nlohmann::json& config);

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

                bool useTime() const;

                units::Distance distanceToTravel() const;

                units::Distance distanceTraveled() const;

                void setDistanceToTravel(const units::Distance& distance);

                units::Velocity baseVelocity() const;

                void setBaseVelocity(const units::Velocity& velocity);

                units::Acceleration maxAcceleration() const;

                void setMaxAcceleration(const units::Acceleration& acceleration);

            protected:
                /* distance to go */
                units::Distance m_distance;
                /* Target speed for driving straight */
                units::Velocity m_velocity;

                /* Maximum acceleration/deceleration */
                units::Acceleration m_max_accel;

                units::Time m_time;

                bool m_direction;

            protected:
                bool m_use_time;

                units::Distance m_init_encoder;
                units::Distance m_distance_travelled;

                std::chrono::time_point<std::chrono::system_clock> m_start_time;
                std::chrono::time_point<std::chrono::system_clock> m_last_time;

                /* Drivetrain subsystem -- used to abstract the control of the drive motors */
                std::shared_ptr<Drivetrain> m_drivetrain;

                /* NavX IMU -- used for gyro & accelerometer */
                std::shared_ptr<NavX> m_navx;

                /* PID Control for rotational correction */
                std::unique_ptr<pid::PidController> m_pid;

                /* Initial yaw -- should try to maintain the same yaw since we are driving straight */
                units::Angle m_initial_yaw;

                bool m_finished;
                bool m_stop;
            };

        }
    }
}

#endif // DRIVE_STRAIGHT_HPP
