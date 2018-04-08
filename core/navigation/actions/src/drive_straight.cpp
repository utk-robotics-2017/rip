#include "navigation_actions/drive_straight.hpp"
#include <misc/logger.hpp>
#include <algorithm>

namespace rip
{
    namespace navigation
    {
        namespace actions
        {
            DriveStraight::DriveStraight(const std::string& name, std::shared_ptr<drivetrains::Drivetrain> drivetrain,
                 std::shared_ptr<navx::NavX> navx, const units::Distance& distance, const units::Velocity& speed,
                 double p, double i, double d, units::Acceleration max_accel)
                : Action(name)
                , m_use_time(false)
                , m_speed(speed)
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
                    l_dynamics.setDeceleration(m_max_accel);
                    r_dynamics.setDeceleration(m_max_accel);

                    std::vector<units::Distance> dists = m_drivetrain->readEncoders(motors);
                    units::Distance threshold = m_distance + m_init_encoder - (1 * units::cm);
                    units::Distance decel_threshold = (m_distance + m_init_encoder) - (m_speed / m_max_accel) * m_speed / 2;

                    l_dynamics.setDistance(m_distance + m_init_encoder - dists[0]);
                    r_dynamics.setDistance(m_distance + m_init_encoder - dists[2]);

                    if(    dists[0] >= threshold || dists[1] >= threshold
                        || dists[2] >= threshold || dists[3] >= threshold )
                    {
                        misc::Logger::getInstance()->debug(
                            "Left: {} {} | Right: {} {} | Target: {}",
                            dists[0].to(units::in), dists[1].to(units::in),
                            dists[2].to(units::in), dists[3].to(units::in),
                            threshold.to(units::in));

                        m_finished = true;
                    }
                    else if(    dists[0] >= decel_threshold || dists[1] >= decel_threshold
                             || dists[2] >= decel_threshold || dists[3] >= decel_threshold )
                    {
                        l_dynamics.setSpeed(0);
                        r_dynamics.setSpeed(0);
                    }
                }
                else
                {
                    m_last_time = std::chrono::system_clock::now();
                    units::Time diff = std::chrono::duration_cast<std::chrono::milliseconds>(m_last_time - m_start_time).count() * units::ms;

                    units::Time threshold = m_time - (0.05 * units::s);

                    if(diff >= threshold)
                    {
                        misc::Logger::getInstance()->debug(
                            "Drove for {} | Target: {}",
                            diff.to(units::s), m_time.to(units::s));
                        m_finished = true;
                    }
                }

                m_drivetrain->drive(l_dynamics, r_dynamics);

            }

            void DriveStraight::setup(nlohmann::json& state)
            {
                using drivetrains::Drivetrain;

                misc::Logger::getInstance()->debug("Driving Straight");
                m_start_time = std::chrono::system_clock::now();
                m_finished = false;

                const std::vector<Drivetrain::Motor> motors = {Drivetrain::Motor::kFrontLeft,
                                                               Drivetrain::Motor::kBackLeft,
                                                               Drivetrain::Motor::kFrontRight,
                                                               Drivetrain::Motor::kBackRight};

               std::vector<units::Distance> dists = m_drivetrain->readEncoders(motors);

               m_init_encoder = dists[0];
               for(units::Distance const &dist : dists) if(dist > m_init_encoder) m_init_encoder = dist;
            }

            void DriveStraight::teardown(nlohmann::json& state)
            {
                m_drivetrain->stop();
            }
        }
    }
}
