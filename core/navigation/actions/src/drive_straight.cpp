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
                if(!m_use_time)
                {
                    std::vector<units::Distance> dists = m_drivetrain->readEncoders(motors);

                    if(    dists[0] >= m_distance || dists[1] >= m_distance
                        || dists[2] >= m_distance || dists[3] >= m_distance )
                    {
                        misc::Logger::getInstance()->debug(
                            "Left: {} {} | Right: {} {} | Target: {}",
                            dists[0].to(units::in), dists[1].to(units::in),
                            dists[2].to(units::in), dists[3].to(units::in),
                            m_distance.to(units::in));

                        m_finished = true;
                        m_drivetrain->stop();
                    }
                    else
                    {
                        m_drivetrain->drive(l_dynamics, r_dynamics);
                    }
                }
                else
                {
                    m_last_time = std::chrono::system_clock::now();
                    units::Time diff = std::chrono::duration_cast<std::chrono::milliseconds>(m_last_time - m_start_time).count() * units::ms;

                    if(diff >= m_time)
                    {
                        misc::Logger::getInstance()->debug(
                            "Drove for {} | Target: {}",
                            diff.to(units::s), m_time.to(units::s));
                        m_finished = true;
                        m_drivetrain->stop();
                    }
                    else
                    {
                        m_drivetrain->drive(l_dynamics, r_dynamics);
                    }
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
