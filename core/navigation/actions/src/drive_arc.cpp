#include "navigation_actions/drive_arc.hpp"

namespace rip
{
    namespace navigation
    {
        namespace actions
        {
            DriveArc::DriveArc(const std::string& name, bool direction,
                std::shared_ptr<drivetrains::Drivetrain> drivetrain,
                const units::AngularVelocity& speed, const units::Angle& angle,
                const units::Distance& radius, const units::Distance& axle_length)
                : Action(name)
                , m_direction(direction)
                , m_drivetrain(drivetrain)
                , m_angular_speed(speed)
                , m_angle(angle)
                , m_radius(radius)
                , m_axle_length(axle_length)
            {
                m_arc_length = angle.to(units::rad) * radius;
                m_linear_speed = radius * speed / units::rad;
            }

            DriveArc::DriveArc(const std::string& name, bool direction,
                std::shared_ptr<drivetrains::Drivetrain> drivetrain,
                const units::Velocity& speed, const units::Distance& arc_length,
                const units::Distance& radius, const units::Distance& axle_length)
                : Action(name)
                , m_direction(direction)
                , m_drivetrain(drivetrain)
                , m_linear_speed(speed)
                , m_arc_length(arc_length)
                , m_radius(radius)
                , m_axle_length(axle_length)
            {
                m_angle = arc_length / radius * units::rad;
                m_angular_speed = speed / radius * units::rad;
            }

            DriveArc::DriveArc(const std::string& name, bool direction,
                std::shared_ptr<drivetrains::Drivetrain> drivetrain,
                const units::Velocity& speed, const units::Distance& arc_length,
                const units::Distance& radius, const units::Distance& axle_length,
                std::shared_ptr<NavX> navx)
                : Action(name)
                , m_direction(direction)
                , m_drivetrain(drivetrain)
                , m_linear_speed(speed)
                , m_arc_length(arc_length)
                , m_radius(radius)
                , m_axle_length(axle_length)
                , m_navx(navx)
            {
                m_angle = arc_length / radius * units::rad;
                m_angular_speed = speed / radius * units::rad;
            }

            bool DriveArc::isFinished()
            {
                m_traveled = readAverageDistance();

                return m_arc_length <= (m_traveled - m_init);
            }

            void DriveArc::update(nlohmann::json& state)
            {
                // todo
                return;
            }

            void DriveArc::setup(nlohmann::json& state)
            {
                std::vector<units::Distance> dist;
                units::Velocity v1, v2;

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

                m_init = readAverageDistance();

                misc::Logger::getInstance()->debug(fmt::format("arc turn intended linear velocity(in/s): {}"
                , m_linear_speed.to(units::in / units::s)));

                misc::Logger::getInstance()->debug(fmt::format("initial(offset) distance(ft): {}", m_init.to(units::ft)));
                motorcontrollers::MotorDynamics dynamicsLeft, dynamicsRight;

                misc::Logger::getInstance()->debug(fmt::format("arc turn intended angular velocity (rev/min): {}"
                , m_angular_speed.to(units::rev / units::minute)));

                misc::Logger::getInstance()->debug(fmt::format("arc turn intended linear velocity(in/s): {}"
                , m_linear_speed.to(units::in / units::s)));

                misc::Logger::getInstance()->debug(fmt::format("arc turn intended linear distance (in): {}"
                , m_arc_length.to(units::in)));

                v1 = m_angular_speed * (m_radius + m_axle_length/2 ) / units::rad;
                v2 = m_angular_speed * (m_radius - m_axle_length/2 ) / units::rad;

                misc::Logger::getInstance()->debug(fmt::format("arc turn outer motor linear velocity (in/s): {}"
                , v1.to(units::in / units::s)));

                misc::Logger::getInstance()->debug(fmt::format("arc turn outer motor linear velocity (in/s): {}"
                , v2.to(units::in / units::s)));

                if(!m_direction) // left turn
                {
                    dynamicsLeft.setSpeed(v2);
                    dynamicsRight.setSpeed(v1);
                }
                else //right turn
                {
                    dynamicsLeft.setSpeed(v1);
                    dynamicsRight.setSpeed(v2);
                }
                m_drivetrain->drive(dynamicsLeft, dynamicsRight);
            }

            units::Distance DriveArc::readAverageDistance()
            {
                std::vector<Motor> motors = {Motor::kFrontLeft, Motor::kFrontRight, Motor::kBackLeft, Motor::kBackRight};
                units::Distance sum=0;
                std::vector<units::Distance> dist = m_drivetrain->readEncoders(motors);
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
