#include "navigation_actions/drive_straight.hpp"
#include <misc/logger.hpp>
#include <algorithm>

#include "navigation_actions/exceptions.hpp"

namespace rip
{
    namespace navigation
    {
        namespace actions
        {
            using drivetrains::Drivetrain;
            using navx::NavX;

            DriveStraight::DriveStraight(const std::string& name, std::shared_ptr<Drivetrain> drivetrain,
                                         std::shared_ptr<NavX> navx, const nlohmann::json& config)
                : TimeoutAction(name, config)
                , m_drivetrain(drivetrain)
                , m_navx(navx)
            {
                if(config.find("use_time") == config.end())
                {
                    throw ActionConfigException("use_time not in config");
                }
                m_use_time = config["use_time"];

                if(m_use_time)
                {
                    if(config.find("time") == config.end())
                    {
                        throw ActionConfigException("time not in config");
                    }
                    m_time = config["time"] * rip::units::s;
                }
                else
                {
                    if(config.find("distance") == config.end())
                    {
                        throw ActionConfigException("distance not in config");
                    }
                    m_distance = config["distance"] * rip::units::in;
                }

                if(config.find("velocity") == config.end())
                {
                    m_velocity = misc::constants::get<double>(misc::constants::kMaxVelocity) * rip::units::in / rip::units::s;
                }
                else
                {
                    m_velocity = config["velocity"] * rip::units::in / rip::units::s;
                }
                m_direction = m_velocity > 0;

                if(config.find("acceleration") != config.end())
                {
                    m_max_accel = config["acceleration"];
                }
                else
                {
                    m_max_accel = misc::constants::get<double>(misc::constants::kMaxAcceleration) * rip::units::in / rip::units::s / rip::units::s;
                }

                if(config.find("stop_after") != config.end())
                {
                    m_stop = config["stop_after"];
                }
                else
                {
                    m_stop = true;
                }

                if(m_navx)
                {
                    double kp = misc::constants::get<double>(misc::constants::kStraightAngleKp);
                    double ki = misc::constants::get<double>(misc::constants::kStraightAngleKi);
                    double kd = misc::constants::get<double>(misc::constants::kStraightAngleKd);
                    m_pid = std::unique_ptr<pid::PidController>(new pid::PidController(m_navx.get(), this, kp, ki, kd));
                    m_pid->setPercentTolerance(1);
                    m_navx->setType(pid::PidInput::Type::kDisplacement);
                    m_pid->setContinuous(true);
                    m_pid->setInputRange(-180 * units::deg(), 180 * units::deg());
                    m_pid->setOutputRange(-2 * units::in() / units::s(), 2 * units::in() / units::s());
                }
            }

            bool DriveStraight::isFinished()
            {
                return m_finished || TimeoutAction::isFinished();
            }

            void DriveStraight::set(double value)
            {
                misc::Logger::getInstance()->debug("set value: {}", value);
            }

            bool DriveStraight::useTime() const
            {
                return m_use_time;
            }

            units::Distance DriveStraight::distanceToTravel() const
            {
                return m_distance;
            }

            units::Distance DriveStraight::distanceTraveled() const
            {
                return m_distance_travelled;
            }

            void DriveStraight::setDistanceToTravel(const units::Distance& distance)
            {
                m_distance = distance;
            }

            units::Velocity DriveStraight::baseVelocity() const
            {
                return m_velocity;
            }

            void DriveStraight::setBaseVelocity(const units::Velocity& velocity)
            {
                m_velocity = velocity;
            }

            units::Acceleration DriveStraight::maxAcceleration() const
            {
                return m_max_accel;
            }

            void DriveStraight::setMaxAcceleration(const units::Acceleration& acceleration)
            {
                m_max_accel = acceleration;
            }

            void DriveStraight::update(nlohmann::json& state)
            {
                const std::vector<Drivetrain::Motor> motors = {
                    Drivetrain::Motor::kFrontLeft,
                    Drivetrain::Motor::kBackLeft,
                    Drivetrain::Motor::kFrontRight,
                    Drivetrain::Motor::kBackRight
                };

                motorcontrollers::MotorDynamics l_dynamics;
                motorcontrollers::MotorDynamics r_dynamics;

                // todo: update these values every loop based on navX
                units::Velocity l_speed = m_velocity;
                units::Velocity r_speed = m_velocity;

                if(m_navx)
                {
                    m_pid->calculate();
                    double pid_correction = m_pid->get();

                    l_speed += pid_correction;
                    r_speed -= pid_correction;

                    units::Velocity v_scale;

                    v_scale = m_direction ? units::max(l_speed, r_speed) : units::min(l_speed, r_speed);

                    // Scale each speed appropriately to not exceed the maximum specified
                    l_speed = (l_speed / v_scale) * m_velocity;
                    r_speed = (r_speed / v_scale) * m_velocity;
                }
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
                        m_velocity.to(units::in / units::s)
                        );

                if(m_navx)
                {
                    misc::Logger::getInstance()->debug("NavX | Yaw: {} | Angle Diff: {}", m_navx->getYaw().to(units::deg), (m_navx->getYaw() - m_initial_yaw).to(units::deg));
                }

                // just for now...
                if (!m_use_time)
                {
                    // get the encoder values from the Roboclaw & determine the greatest value
                    std::vector<units::Distance> dists = m_drivetrain->readEncoders(motors);
                    m_distance_travelled = *(std::max_element(dists.begin(), dists.end()));

                    // todo: Distance PID
                    // Average?
                    // set a threshold for stopping -- this tries to account for the delay in actually stopping
                    // this is quite arbitrary right now but should be kinda close
                    units::Distance threshold = m_distance + m_init_encoder - (m_velocity * (0.1 * units::s));

                    state["current_distance"] = m_distance_travelled;
                    state["distance_threshold"] = threshold;

                    // going forward encoder > threshold
                    // going backwards encoder < threshold
                    if ( (m_direction && m_distance_travelled >= threshold) || (!m_direction && m_distance_travelled <= threshold))
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

                m_drivetrain->drive(l_dynamics, r_dynamics);
            }

            void DriveStraight::setup(nlohmann::json& state)
            {
                misc::Logger::getInstance()->debug("Driving Straight");
                m_start_time = std::chrono::system_clock::now();
                m_finished = false;

                const std::vector<Drivetrain::Motor> motors = {
                    Drivetrain::Motor::kFrontLeft,
                    Drivetrain::Motor::kBackLeft,
                    Drivetrain::Motor::kFrontRight,
                    Drivetrain::Motor::kBackRight
                };


                // Drivetrain Encoders
                std::vector<units::Distance> dists = m_drivetrain->readEncoders(motors);
                m_init_encoder = *(std::max_element(dists.begin(), dists.end()));

                state["initial_encoder"] = m_init_encoder;

                // NavX
                if(m_navx)
                {
                    m_initial_yaw = m_navx->getYaw();
                    misc::Logger::getInstance()->debug("Inital Yaw: {}", m_initial_yaw.to(units::deg));

                    state["initial_yaw"] = m_initial_yaw;

                    m_pid->setSetpoint(m_initial_yaw.to(units::deg));
                    m_pid->enable(); // Not sure if this does anything...

                }
                state["direction"] = m_direction;


                TimeoutAction::setup(state);
            }

            void DriveStraight::teardown(nlohmann::json& state)
            {
                if(m_stop)
                {
                    m_drivetrain->stop();
                }
            }
        }
    }
}
