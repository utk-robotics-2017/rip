#include "path_follower_gui/robot.hpp"

#include <QFile>

#include <fmt/format.h>

#include "path_follower_gui/exceptions.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            namespace gui
            {
                void Robot::draw(QPainter& painter, const RigidTransform2d& pose) const
                {
                    painter.setBrush(Qt::red);

                    geometry::Point center = m_shape.centroid();

                    geometry::Point translation = pose.translation();
                    units::Angle rotation = pose.rotation().angle();

                    QPainterPath path;
                    geometry::Point point = m_shape.front();

                    units::Distance magnitude = point.distance(center);
                    units::Angle current_angle = geometry::atan(point - center);

                    point = geometry::Point(magnitude * units::cos(current_angle + rotation), magnitude * units::sin(current_angle + rotation)) + center + translation;

                    path.moveTo(point.x()(), point.y()());
                    for(geometry::Point p : m_shape)
                    {
                        units::Distance magnitude = p.distance(center);
                        units::Angle current_angle = geometry::atan(p - center);
                        point = geometry::Point(magnitude * units::cos(current_angle + rotation), magnitude * units::sin(current_angle + rotation)) + center + translation;
                        path.lineTo(point.x()(), point.y()());
                    }

                    painter.drawPath(path);
                }

                std::string Robot::name() const
                {
                    return m_name;
                }

                void Robot::setName(const std::string& name)
                {
                    m_name = name;
                }

                geometry::Polygon Robot::shape() const
                {
                    return m_shape;
                }

                void Robot::setShape(const geometry::Polygon& shape)
                {
                    m_shape = shape;
                }

                void Robot::setPoint(int index, const geometry::Point& point)
                {
                    assert(index >= 0 && index < m_shape.size());
                    m_shape[index] = point;
                }

                units::Velocity Robot::maxSpeed() const
                {
                    return m_max_speed;
                }

                void Robot::setMaxSpeed(const units::Velocity& max_speed)
                {
                    m_max_speed = max_speed;
                }

                units::Acceleration Robot::maxAcceleration() const
                {
                    return m_max_acceleration;
                }

                void Robot::setMaxAcceleration(const units::Acceleration& max_acceleration)
                {
                    m_max_acceleration = max_acceleration;
                }

                units::Distance Robot::wheelbase() const
                {
                    return m_wheelbase;
                }

                void Robot::setWheelbase(const units::Distance& wheelbase)
                {
                    m_wheelbase = wheelbase;
                }

                void Robot::save(const QString& filepath) const
                {
                    QFile f(filepath);
                    if (f.open(QIODevice::WriteOnly))
                    {
                        nlohmann::json j;
                        j["name"] = m_name;
                        j["shape"] = m_shape;
                        nlohmann::json config;
                        config["max_speed"] = m_max_speed;
                        config["max_acceleration"] = m_max_acceleration;
                        config["wheelbase"] = m_wheelbase;
                        j["config"] = config;
                        f.write(j.dump(4).c_str());
                    }
                    else
                    {
                        throw RobotConfigException(fmt::format("Cannot save robot to file: {}", filepath.toStdString()));
                    }
                }

                std::shared_ptr<Robot> Robot::load(const QString& filepath)
                {
                    QFile f(filepath);
                    if (f.open(QIODevice::ReadOnly))
                    {
                        nlohmann::json j = nlohmann::json::parse(f.readAll());

                        std::shared_ptr<Robot> robot = std::make_shared<Robot>();

                        if(j.find("name") == j.end())
                        {
                            throw RobotConfigException("Missing name");
                        }
                        robot->m_name = j["name"];

                        if(j.find("shape") == j.end())
                        {
                            throw RobotConfigException("Missing shape");
                        }
                        robot->m_shape = j["shape"].get<geometry::Polygon>();

                        if(j.find("config") == j.end())
                        {
                            throw RobotConfigException("Missing config");
                        }

                        nlohmann::json config = j["config"];

                        if(config.find("max_speed") == config.end())
                        {
                            throw RobotConfigException("Missing max_speed");
                        }
                        robot->m_max_speed = config["max_speed"];

                        if(config.find("max_acceleration") == config.end())
                        {
                            throw RobotConfigException("Missing max_acceleration");
                        }
                        robot->m_max_acceleration = config["max_acceleration"];

                        if(config.find("wheelbase") == config.end())
                        {
                            throw RobotConfigException("Missing wheelbase");
                        }
                        robot->m_wheelbase = config["wheelbase"];

                        return robot;
                    }
                    throw FileNotFound(fmt::format("Could not find robot file: {}", filepath.toStdString()));
                }

            }
        }
    }
}
