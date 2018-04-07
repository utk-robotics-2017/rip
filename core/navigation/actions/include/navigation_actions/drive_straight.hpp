#ifndef DRIVE_STRAIGHT_HPP
#define DRIVE_STRAIGHT_HPP

#include <json.hpp>

#include <framework/action.hpp>
#include <drivetrains/drivetrain.hpp>
#include <navx/navx.hpp>

#include <chrono>


namespace rip
{
    namespace navigation
    {
        namespace actions
        {
            class DriveStraight : public framework::Action
            {
            public:
                DriveStraight(const std::string& name, std::shared_ptr<drivetrains::Drivetrain> drivetrain,
                     std::shared_ptr<navx::NavX> navx, const units::Distance& distance, double p, double i, double d, units::Acceleration max_accel);

                DriveStraight(const std::string& name, std::shared_ptr<drivetrains::Drivetrain> drivetrain,
                     std::shared_ptr<navx::NavX> navx, const units::Time& time, const units::Velocity& speed, units::Acceleration max_accel);

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

                /* distance to go */
                units::Distance m_distance;

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

                /* Initial yaw -- should try to maintain the same yaw since we are driving straight */
                units::Angle m_initial_yaw;

                bool m_finished;
            };

        }
    }
}

#endif // DRIVE_STRAIGHT_HPP
