#include "navigation_actions/drive_arc.hpp"

namespace rip
{
    namespace navigation
    {
        namespace actions
        {
            DriveArc::DriveArc(const std::string& name, bool right,
                std::shared_ptr<drivetrains::Drivetrain> drivetrain,
                const units::AngularVelocity& speed, const units::Angle& angle,
                const units::Distance& radius, const units::Distance& axle_length)
                : Action(name)
                , m_right(right)
                , m_drivetrain(drivetrain)
                , m_angular_speed(speed)
                , m_angle(angle)
                , m_radius(radius)
                , m_axle_length(axle_length)
            {
                printf("hi7\n");

                m_arc_length = angle.to(units::rad) * radius;
                m_linear_speed = radius * speed / units::rad;
            }

            DriveArc::DriveArc(const std::string& name, bool right,
                std::shared_ptr<drivetrains::Drivetrain> drivetrain,
                const units::Velocity& speed, const units::Distance& arc_length,
                const units::Distance& radius, const units::Distance& axle_length)
                : Action(name)
                , m_right(right)
                , m_drivetrain(drivetrain)
                , m_linear_speed(speed)
                , m_arc_length(arc_length)
                , m_radius(radius)
                , m_axle_length(axle_length)
            {
                printf("hi7\n");

                m_angle = arc_length / radius * units::rad;
                m_angular_speed = speed / radius * units::rad;
            }

            bool DriveArc::isFinished()
            {
                m_traveled = readAverageDistance();
                return m_arc_length <= (m_traveled - m_init_dist);
            }

            void DriveArc::update(nlohmann::json& state)
            {
                /*
                std::chrono::time_point<std::chrono::system_clock> current = std::chrono::system_clock::now();
                units::Time diff = std::chrono::duration_cast<std::chrono::milliseconds>(current - m_start_time).count() * units::s;
                std::array<Motor, 2> motors;
                std::array<motorcontrollers::MotorDynamics, 2> dynamics;
                double diff_state, actual_state, expected_state;
                */
                /**
                 * Units to be used on the json (since units cannot be serialized)
                 * Distance: cm
                 * Time: seconds
                 * Angle: degrees
                 *
                if(!m_right)
                {
                    motors[0] = Motor::kFrontLeft;
                    motors[1] = Motor::kFrontRight;
                }
                else
                {
                    motors[0] = Motor::kFrontRight;
                    motors[1] = Motor::kFrontLeft;
                }
                // get actual state and expected state in pairs
                state[this->name()] = {
                    {"encoderDistance", {
                        {"expected", (m_linear_speed * diff).to(units::cm)},
                        {"actual", readAverageDistance()}
                    }},
                    {"angle" , {
                        {"expected", (m_angular_speed * diff).to(units::deg)},
                        {"actual", (m_navx->getAngle() - m_init_angle).to(units::deg)}
                    }},
                    {"angularVelocity", {
                        {"expected", m_angular_speed.to(units::deg / units::s)},
                        {"actual", m_navx->getRate().to(units::deg / units::s)}
                    }},
                    {"linearVelocity", {
                        {"inner", {
                            {"expected", m_velocity_inner.to(units::cm / units::s)},
                            {"actual", m_drivetrain->readEncoderVelocity(motors[0]).to(units::cm / units::s)}
                        }},
                        {"outer", {
                            {"expected", m_velocity_outer.to(units::cm / units::s)},
                            {"actual", m_drivetrain->readEncoderVelocity(motors[1]).to(units::cm / units::s)}
                        }},
                        {"average", {
                            {"expected", m_linear_speed.to(units::cm / units::s)},
                            {"actual", (m_drivetrain->readEncoderVelocity(motors[0]) +
                            m_drivetrain->readEncoderVelocity(motors[1])).to(units::cm / units::s)/2 }
                        }}
                }}};

                // adjustments
                actual_state = state[this->name()]["encoderDistance"]["actual"];
                expected_state = state[this->name()]["encoderDistance"]["expected"];
                diff_state = actual_state - expected_state;

                // Avg Speed adjustment
                if(diff_state > k_dead_zone * expected_state) //actual value larger than expected
                {
                    for(int i=0; i<2; i++)
                    {
                        m_adjusted_speeds[i] -= .0001;
                        dynamics[i].setSpeed(m_adjusted_speeds[i]);
                    }
                    m_drivetrain->drive(dynamics[0], dynamics[1]);
                }
                else if(diff_state < k_dead_zone * expected_state * -1)//smaller or equal than expected
                {
                    for(int i=0; i<2; i++)
                    {
                        m_adjusted_speeds[i] += .0001;
                        dynamics[i].setSpeed(m_adjusted_speeds[i]);
                    }
                    m_drivetrain->drive(dynamics[0], dynamics[1]);
                }
                
                // yaw adjustment
                actual_state = state[this->name()]["angle"]["actual"];
                expected_state = state[this->name()]["angle"]["expected"];
                diff_state = actual_state - expected_state;

                if(diff_state > k_dead_zone * expected_state) //angle greater than expected
                {
                    m_adjusted_speeds[0] -= .0001 * units::cm / units::s;
                    m_adjusted_speeds[1] += .0001 * units::cm / units::s;
                    dynamics[0].setSpeed(m_adjusted_speeds[0]);
                    dynamics[1].setSpeed(m_adjusted_speeds[1]);
                    m_drivetrain->drive(dynamics[0], dynamics[1]);
                }
                
                else if(diff_state < k_dead_zone * expected_state * -1)//smaller or equal than expected
                {
                    m_adjusted_speeds[1] += .0001 * units::cm / units::s;
                    m_adjusted_speeds[0] -= .0001 * units::cm / units::s;
                    dynamics[0].setSpeed(m_adjusted_speeds[0]);
                    dynamics[1].setSpeed(m_adjusted_speeds[1]);
                    m_drivetrain->drive(dynamics[0], dynamics[1]);
                }
                */
            }

            void DriveArc::setup(nlohmann::json& state)
            {
                std::vector<units::Distance> dist;
                motorcontrollers::MotorDynamics dynamicsLeft, dynamicsRight;

                if(m_arc_length <= 0)
                {
                    throw OutofBoundsException("arc_length should be greater than 0");
                }
                if(m_radius < m_axle_length)
                {
                    throw OutofBoundsException("radius should be greater than the axle length");
                }
                if(m_axle_length <= 0)
                {
                    throw OutofBoundsException("axle length should be positive");
                }

                m_start_time = std::chrono::system_clock::now();
                m_init_dist = readAverageDistance();

                /*
                do
                {
                    m_init_angle = m_navx->getAngle();
                } while(m_init_angle.to(units::deg) == 0);
                */
                misc::Logger::getInstance()->debug(fmt::format("arc turn intended linear velocity(in/s): {}"
                , m_linear_speed.to(units::in / units::s)));

                misc::Logger::getInstance()->debug(fmt::format("initial(offset) distance(ft): {}", m_init_dist.to(units::ft)));

                misc::Logger::getInstance()->debug(fmt::format("arc turn intended angular velocity (rev/min): {}"
                , m_angular_speed.to(units::rev / units::minute)));

                misc::Logger::getInstance()->debug(fmt::format("arc turn intended linear velocity(in/s): {}"
                , m_linear_speed.to(units::in / units::s)));

                misc::Logger::getInstance()->debug(fmt::format("arc turn intended linear distance (in): {}"
                , m_arc_length.to(units::in)));

                m_velocity_outer = m_angular_speed * (m_radius + m_axle_length/2 ) / units::rad;
                m_velocity_inner =  m_angular_speed * (m_radius - m_axle_length/2 ) / units::rad;
                m_adjusted_speeds[0] = m_velocity_inner;
                m_adjusted_speeds[1] = m_velocity_outer;

                misc::Logger::getInstance()->debug(fmt::format("arc turn outer motor linear velocity (in/s): {}"
                , m_velocity_outer.to(units::in / units::s)));

                misc::Logger::getInstance()->debug(fmt::format("arc turn outer motor linear velocity (in/s): {}"
                , m_velocity_inner.to(units::in / units::s)));

                if(!m_right) // left turn
                {
                    dynamicsLeft.setSpeed(m_velocity_inner);
                    dynamicsRight.setSpeed(m_velocity_outer);
                }
                else //right turn
                {
                    dynamicsLeft.setSpeed(m_velocity_outer);
                    dynamicsRight.setSpeed(m_velocity_inner);
                }
                m_drivetrain->drive(dynamicsLeft, dynamicsRight);
            }

            units::Distance DriveArc::readAverageDistance()
            {
                units::Distance sum=0;
                std::vector<units::Distance> dist = m_drivetrain->readEncoders({Motor::kFrontLeft,
                     Motor::kFrontRight, Motor::kBackLeft, Motor::kBackRight});
                for(int i=0; i<static_cast<int>(dist.size()); i++)
                {
                    sum += dist[i];
                }
                sum /= dist.size();
                return sum;
            }

            void DriveArc::teardown(nlohmann::json& state)
            {
                // todo
                m_drivetrain->stop();
            }
        }
    }
}
