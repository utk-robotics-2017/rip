#include "navigation_actions/drive_straight.hpp"
#include <misc/logger.hpp>
#include <algorithm>

namespace rip
{
    namespace navigation
    {
        namespace actions
        {
            using drivetrains::Drivetrain;
			using navx::NavX;
            
			DriveStraight::DriveStraight(const std::string& name, std::shared_ptr<Drivetrain> drivetrain,
                 std::shared_ptr<NavX> navx, const units::Distance& distance, const units::Velocity& speed,
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

            DriveStraight::DriveStraight(const std::string& name, std::shared_ptr<Drivetrain> drivetrain,
                 std::shared_ptr<NavX> navx, const units::Time& time, const units::Velocity& speed,
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
					// get the encoder values from the Roboclaw & determine the greatest value
                    std::vector<units::Distance> dists = m_drivetrain->readEncoders(motors);
					units::Distance max_encoder = *(std::max_element(dists.begin(), dists.end()));

					// set a threshold for stopping -- this tries to account for the delay in actually stopping
					// this is quite arbitrary right now but should be kinda close
                    units::Distance threshold = m_distance + m_init_encoder - (m_speed * (0.06 * units::s));

                    if(max_encoder >= threshold)
                    {
                        misc::Logger::getInstance()->debug(
                            "Left: {} {} | Right: {} {} | Target: {}",
                            dists[0].to(units::in), dists[1].to(units::in),
                            dists[2].to(units::in), dists[3].to(units::in),
                            threshold.to(units::in));

                        m_finished = true;
                    }
                }
                else
                {
                    m_last_time = std::chrono::system_clock::now();
                    units::Time diff = std::chrono::duration_cast<std::chrono::milliseconds>(m_last_time - m_start_time).count() * units::ms;

                    units::Time threshold = m_time - (0.06 * units::s);

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
				misc::Logger::getInstance()->debug("Driving Straight");
                m_start_time = std::chrono::system_clock::now();
                m_finished = false;

                const std::vector<Drivetrain::Motor> motors = {Drivetrain::Motor::kFrontLeft,
                                                               Drivetrain::Motor::kBackLeft,
                                                               Drivetrain::Motor::kFrontRight,
                                                               Drivetrain::Motor::kBackRight};

				std::vector<units::Distance> dists = m_drivetrain->readEncoders(motors);

				m_init_encoder = *(std::max_element(dists.begin(), dists.end()));
            }

            void DriveStraight::teardown(nlohmann::json& state)
            {
                m_drivetrain->stop();
            }
        }
    }
}
