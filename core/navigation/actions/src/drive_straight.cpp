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
                                         std::shared_ptr<navx::NavX> navx, const units::Time& time, const units::Velocity& speed,
                                         units::Acceleration max_accel, double p, double i, double d, bool forward=true)
                : Action(name)
                , m_use_time(true)
                , m_time(time)
                , m_speed(speed)
                , m_max_accel(max_accel)
                , m_drivetrain(drivetrain)
                , m_pid(new pid::PidController(navx.get(), this, p, i , d))
                , m_direction(forward)
            {
                m_pid->setSetpoint(0);
                m_pid->setTolerance(10);
                m_navx->setType(pid::PidInput::Type::kDisplacement);
                m_pid->setContinuous(true);
                m_pid->setInputRange(-180 * units::deg(), 180 * units::deg());
                m_pid->setOutputRange(-2 * units::in(), 2 * units::in());
            }

            DriveStraight::DriveStraight(const std::string& name, std::shared_ptr<Drivetrain> drivetrain,
                                         std::shared_ptr<NavX> navx, const units::Distance& distance, const units::Velocity& speed,
                                         units::Acceleration max_accel, double p, double i, double d)
                : Action(name)
                , m_use_time(false)
                , m_speed(speed)
                , m_distance(distance)
                , m_max_accel(max_accel)
                , m_drivetrain(drivetrain)
                , m_navx(navx)
                , m_pid(new pid::PidController(navx.get(), this, p, i , d))
                , m_direction(speed > 0)
            {
                m_pid->setSetpoint(0);
                m_pid->setTolerance(10);
                m_navx->setType(pid::PidInput::Type::kDisplacement);
                m_pid->setContinuous(true);
                m_pid->setInputRange(-180 * units::deg(), 180 * units::deg());
                m_pid->setOutputRange(-2 * units::in(), 2 * units::in());
            }

            bool DriveStraight::isFinished()
            {
                return m_finished;
            }

            void DriveStraight::set(double value)
            {
                misc::Logger::getInstance()->debug("set value: {}", value);
            }

            void DriveStraight::update(nlohmann::json& state)
            {
                const std::vector<Drivetrain::Motor> motors = {Drivetrain::Motor::kFrontLeft,
                                                               Drivetrain::Motor::kBackLeft,
                                                               Drivetrain::Motor::kFrontRight,
                                                               Drivetrain::Motor::kBackRight
                                                              };

                motorcontrollers::MotorDynamics l_dynamics;
                motorcontrollers::MotorDynamics r_dynamics;

                // todo: update these values every loop based on navX
                units::Velocity l_speed = m_speed;
                units::Velocity r_speed = m_speed;

                l_dynamics.setSpeed(l_speed);
                r_dynamics.setSpeed(r_speed);

                l_dynamics.setAcceleration(m_max_accel);
                r_dynamics.setAcceleration(m_max_accel);

                state["left_target_speed"] = l_speed;
                state["right_target_speed"] = r_speed;

                std::vector<units::Velocity> vel = m_drivetrain->readEncoderVelocities(motors);

                misc::Logger::getInstance()->debug(
                            "Encoder Velocity | Left: {} {} | Right: {} {} | Target: {}",
                            vel[0].to(units::in / units::s), vel[1].to(units::in / units::s),
                            vel[2].to(units::in / units::s), vel[3].to(units::in / units::s),
                            m_speed.to(units::in / units::s)
                            );

                misc::Logger::getInstance()->debug(
                            "NavX | Velocity: X: {} Y: {} Z: {} | Yaw: {} | Angle: {} | Angle Diff: {} | Fused Heading: {}",
                            m_navx->getVelocityX().to(units::in / units::s),
                            m_navx->getVelocityY().to(units::in / units::s),
                            m_navx->getVelocityZ().to(units::in / units::s),
                            m_navx->getYaw().to(units::deg),
                            m_navx->getAngle().to(units::deg),
                            (m_navx->getAngle() - m_initial_yaw).to(units::deg),
                            m_navx->getFusedHeading().to(units::deg)
                            );

                // just for now...
                if (!m_use_time)
                {
                    // get the encoder values from the Roboclaw & determine the greatest value
                    std::vector<units::Distance> dists = m_drivetrain->readEncoders(motors);
                    units::Distance max_encoder = *(std::max_element(dists.begin(), dists.end()));

                    // set a threshold for stopping -- this tries to account for the delay in actually stopping
                    // this is quite arbitrary right now but should be kinda close
                    units::Distance threshold = m_distance + m_init_encoder - (m_speed * (0.1 * units::s));

                    state["current_distance"] = max_encoder;
                    state["distance_threshold"] = threshold;

                    // going forward encoder > threshold
                    // going backwards encoder < threshold
                    if ( (m_direction && max_encoder >= threshold) || (!m_direction && max_encoder <= threshold))
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

                    units::Time threshold = m_time - (0.1 * units::s);

                    if (diff >= threshold)
                    {
                        misc::Logger::getInstance()->debug(
                                    "Drove for {} | Target: {}",
                                    diff.to(units::s), m_time.to(units::s));
                        m_finished = true;
                    }
                }

                if (!m_finished)
                {
                    m_drivetrain->drive(l_dynamics, r_dynamics);
                }
                else
                {
                    m_drivetrain->stop();
                }

                m_pid->calculate();
            }

            void DriveStraight::setup(nlohmann::json& state)
            {
                misc::Logger::getInstance()->debug("Driving Straight");
                m_start_time = std::chrono::system_clock::now();
                m_finished = false;

                const std::vector<Drivetrain::Motor> motors = {Drivetrain::Motor::kFrontLeft,
                                                               Drivetrain::Motor::kBackLeft,
                                                               Drivetrain::Motor::kFrontRight,
                                                               Drivetrain::Motor::kBackRight
                                                              };


                // Drivetrain Encoders
                std::vector<units::Distance> dists = m_drivetrain->readEncoders(motors);
                m_init_encoder = *(std::max_element(dists.begin(), dists.end()));

                state["initial_encoder"] = m_init_encoder;

                // NavX
                m_navx->zeroYaw();
                m_initial_yaw = m_navx->getYaw();
                misc::Logger::getInstance()->debug(
                            "Inital Yaw: {}", m_initial_yaw.to(units::deg)
                            );

                state["initial_yaw"] = m_initial_yaw;
                state["direction"] = m_direction;
            }

            void DriveStraight::teardown(nlohmann::json& state)
            {
                // also called in update -- leaving this here for now?
                m_drivetrain->stop();
            }
        }
    }
}
