#include "navigation_actions/drive_straight.hpp"
#include <misc/logger.hpp>
namespace rip
{
    namespace navigation
    {
        namespace actions
        {
            DriveStraight::DriveStraight(const std::string& name, std::shared_ptr<drivetrains::Drivetrain> drivetrain,
                 std::shared_ptr<navx::NavX> navx, const units::Distance& distance, double p, double i, double d,
                 units::Acceleration max_accel)
                : Action(name)
                , m_use_time(false)
                , m_speed(1.0 * units::m / units::s)
                , m_distance(distance)
                , m_time(distance / m_speed)
                , m_max_accel(max_accel)
                , m_drivetrain(drivetrain)
                , m_navx(navx)
            {}

            DriveStraight::DriveStraight(const std::string& name, std::shared_ptr<drivetrains::Drivetrain> drivetrain,
                 std::shared_ptr<navx::NavX> navx, const units::Time& time, const units::Velocity& speed,
                 units::Acceleration max_accel)
                : Action(name)
                , m_use_time(true)
                , m_time(time)
                , m_speed(speed)
                , m_distance(speed * time)
                , m_max_accel(max_accel)
                , m_drivetrain(drivetrain)
                , m_navx(navx)
            {}

            bool DriveStraight::isFinished()
            {
                return m_finished;
            }

            void DriveStraight::update(nlohmann::json& state)
            {
                using drivetrains::Drivetrain;

                const std::vector<Drivetrain::Motor> motors = {Drivetrain::Motor::kFrontLeft,
                                                               Drivetrain::Motor::kBackLeft,
                                                               Drivetrain::Motor::kFrontRight,
                                                               Drivetrain::Motor::kBackRight};

                motorcontrollers::MotorDynamics l_dynamics;
                motorcontrollers::MotorDynamics r_dynamics;

                // todo: update these values every loop based on navX
                units::Velocity l_speed = m_speed;
                units::Velocity r_speed = m_speed;

                l_dynamics.setSpeed(l_speed);
                r_dynamics.setSpeed(r_speed);

                l_dynamics.setAcceleration(m_max_accel);
                r_dynamics.setAcceleration(m_max_accel);

                // just for now...
                std::vector<units::Distance> dists = m_drivetrain->readEncoders(motors);

                if(    dists[0] >= m_distance || dists[1] >= m_distance
                    || dists[2] >= m_distance || dists[3] >= m_distance )
                {
                    m_finished = true;
                    m_drivetrain->stop();
                }
                else
                {
                    m_drivetrain->drive(l_dynamics, r_dynamics);
                }
            }

            void DriveStraight::setup(nlohmann::json& state)
            {
                misc::Logger::getInstance()->debug("Driving Straight");
                m_start_time = std::chrono::system_clock::now();
                m_finished = false;
            }

            void DriveStraight::teardown(nlohmann::json& state)
            {
                m_drivetrain->stop();
            }
        }
    }
}
