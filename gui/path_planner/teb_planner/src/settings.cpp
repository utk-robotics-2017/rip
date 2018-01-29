#include <teb_planner_gui/settings.hpp>

#include <QStandardPaths>
#include <QDir>

#include <teb_planner/point_obstacle.hpp>
#include <teb_planner/line_obstacle.hpp>
#include <teb_planner/polygon_obstacle.hpp>

#include <teb_planner_gui/exceptions.hpp>
#include <fmt/format.h>

namespace rip
{
    namespace gui
    {
        namespace tebplanner
        {
            std::shared_ptr<Settings> Settings::m_singleton = nullptr;

            std::shared_ptr<Settings> Settings::getInstance()
            {
                if (!m_singleton)
                {
                    m_singleton = std::shared_ptr<Settings>(new Settings);
                }
                return m_singleton;
            }

            Settings::~Settings()
            {
                //save();
            }

            void Settings::load()
            {
                QString save_location = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
                QDir sl(save_location);
                if(!sl.exists())
                {
                    sl.mkpath(".");
                }

                {
                    // Load all the robot configs
                    QString robot_save_location = save_location + QDir::separator() + "robots";
                    QDir robot_dir(robot_save_location);
                    if (robot_dir.exists())
                    {
                        QStringList files = robot_dir.entryList(QDir::Files);
                        for (const QString& filename : files)
                        {
                            QFile f(robot_save_location + QDir::separator() + filename);
                            if (f.open(QIODevice::ReadOnly))
                            {
                                nlohmann::json j = nlohmann::json::parse(f.readAll());
                                m_robots[j["name"]] = std::make_shared<navigation::PolygonRobotFootprintModel>(j["footprint"].get<geometry::Polygon>());
                            }
                        }
                    }
                    else
                    {
                        robot_dir.mkpath(".");
                    }
                }

                {
                    // Load all the obstacles configs
                    QString obstacles_save_location = save_location + QDir::separator() + "obstacles";
                    QDir obstacles_dir(obstacles_save_location);
                    if (obstacles_dir.exists())
                    {
                        QStringList files = obstacles_dir.entryList(QDir::Files);
                        for (const QString& filename : files)
                        {
                            QFile f(obstacles_save_location + QDir::separator() + filename);
                            if (f.open(QIODevice::ReadOnly))
                            {
                                nlohmann::json j = nlohmann::json::parse(f.readAll());

                                std::vector< std::shared_ptr< navigation::Obstacle > > obstacles;

                                for(const nlohmann::json& v : j["obstacles"])
                                {
                                    if(v.find("polygon") != v.end())
                                    {
                                        obstacles.push_back(std::make_shared<navigation::PolygonObstacle>(v["polygon"].get<geometry::Polygon>()));
                                    }
                                    else if(v.find("start") != v.end())
                                    {
                                        obstacles.push_back(std::make_shared<navigation::LineObstacle>(v["start"].get<geometry::Point>(), v["end"].get<geometry::Point>()));
                                    }
                                    else if(v.find("point") != v.end())
                                    {
                                        obstacles.push_back(std::make_shared<navigation::PointObstacle>(v["point"].get<geometry::Point>()));
                                    }
                                    else
                                    {
                                        throw UnknownObstacle("Unknown obstacle");
                                    }
                                }

                                m_obstacles[j["name"]] = std::make_shared< std::vector< std::shared_ptr<navigation::Obstacle> > >(obstacles);

                            }
                        }
                    }
                    else
                    {
                        obstacles_dir.mkpath(".");
                    }
                }

                {
                    // Load Teb Config
                    QString config_save_location = save_location + QDir::separator() + "config";
                    QDir config_dir(config_save_location);
                    if(config_dir.exists())
                    {
                        QStringList files = config_dir.entryList(QDir::Files);
                        for(const QString& filename : files)
                        {
                            QFile f(config_save_location + QDir::separator() + filename);
                            if(f.open(QIODevice::ReadOnly))
                            {
                                nlohmann::json j = nlohmann::json::parse(f.readAll());
                                m_config[j["name"]] = std::make_shared<navigation::TebConfig>(j["config"].get<navigation::TebConfig>());
                            }
                        }
                    }
                    else
                    {
                        config_dir.mkpath(".");
                    }
                }
            }

            void Settings::save() const
            {
                QString save_location = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);

                if(m_robots.size())
                {
                    // Save robot footprint
                    for (auto iter : m_robots)
                    {
                        QString filepath = save_location + QDir::separator() + "robots" + QDir::separator() + QString::fromStdString(iter.first) + ".json";
                        QFile f(filepath);
                        if (f.open(QIODevice::WriteOnly))
                        {
                            nlohmann::json j;
                            j["name"] = iter.first;
                            j["footprint"] = iter.second->polygon();
                            f.write(j.dump().c_str());
                        }
                    }
                }

                if(m_obstacles.size())
                {
                    // Save Obstacles
                    for(auto iter : m_obstacles)
                    {
                        QString filepath = save_location + QDir::separator() + "obstacles" + QDir::separator() + QString::fromStdString(iter.first) + ".json";
                        QFile f(filepath);
                        if(f.open(QIODevice::WriteOnly))
                        {
                            nlohmann::json j;
                            j["name"] = iter.first;
                            for(std::shared_ptr<navigation::Obstacle> obstacle : *(iter.second))
                            {
                                nlohmann::json o;
                                std::shared_ptr<navigation::PointObstacle> point = std::dynamic_pointer_cast<navigation::PointObstacle>(obstacle);
                                if(point)
                                {
                                    o["point"] = point->centroid();
                                }

                                std::shared_ptr<navigation::LineObstacle> line = std::dynamic_pointer_cast<navigation::LineObstacle>(obstacle);
                                if(line)
                                {
                                    o["start"] = line->start();
                                    o["end"] = line->end();
                                }

                                std::shared_ptr<navigation::PolygonObstacle> polygon = std::dynamic_pointer_cast<navigation::PolygonObstacle>(obstacle);
                                if(polygon)
                                {
                                    o["polygon"] = polygon->polygon();
                                }

                                j["obstacles"].push_back(o);
                            }
                            f.write(j.dump().c_str());
                        }
                    }
                }

                if(m_config.size())
                {
                    // Save teb config
                    for(auto iter : m_config)
                    {
                        QString filepath = save_location + QDir::separator() + "config" + QDir::separator() + QString::fromStdString(iter.first) + ".json";
                        QFile f(filepath);
                        if(f.open(QIODevice::WriteOnly))
                        {
                            nlohmann::json j;
                            j["name"] = iter.first;
                            j["config"] = *(iter.second);
                            f.write(j.dump().c_str());
                        }
                    }
                }
            }

            std::shared_ptr<navigation::PolygonRobotFootprintModel> Settings::addRobot(const std::string& name)
            {
                m_robots[name] = std::make_shared<navigation::PolygonRobotFootprintModel>(geometry::Polygon());
                return m_robots[name];
            }

            void Settings::removeRobot(const std::string& name)
            {
                if (m_robots.find(name) != m_robots.end())
                {
                    m_robots.erase(name);
                }
            }

            std::shared_ptr<navigation::PolygonRobotFootprintModel> Settings::robot(const std::string& name) const
            {
                if (m_robots.find(name) == m_robots.end())
                {
                    return nullptr;
                }
                return m_robots.at(name);
            }

            void Settings::saveRobot(const std::string& name) const
            {
                if (m_robots.find(name) != m_robots.end())
                {
                    QString save_location = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
                    QString filepath = save_location + QDir::separator() + "robots" + QDir::separator() + QString::fromStdString(name) + ".json";
                    QFile f(filepath);
                    if (f.open(QIODevice::WriteOnly))
                    {
                        nlohmann::json j;
                        j["name"] = name;
                        j["footprint"] = m_robots.at(name)->polygon();
                        f.write(j.dump().c_str());
                    }
                }
            }

            void Settings::loadRobot(const std::string& name)
            {
                QString save_location = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);

                QString robot_save_location = save_location + QDir::separator() + "robots";
                QDir robot_dir(robot_save_location);
                if (robot_dir.exists())
                {
                    QString filename = QString::fromStdString(name);
                    QFile f(robot_save_location + QDir::separator() + filename);
                    if (f.open(QIODevice::ReadOnly))
                    {
                        nlohmann::json j = nlohmann::json::parse(f.readAll());
                        m_robots[name] = std::make_shared<navigation::PolygonRobotFootprintModel>(j["footprint"].get<geometry::Polygon>());
                    }
                }
            }

            std::shared_ptr< std::vector< std::shared_ptr<navigation::Obstacle> > > Settings::addObstacles(const std::string& name)
            {
                m_obstacles[name] = std::make_shared< std::vector< std::shared_ptr<navigation::Obstacle> > >();
                return m_obstacles[name];
            }

            void Settings::removeObstacles(const std::string& name)
            {
                if (m_obstacles.find(name) != m_obstacles.end())
                {
                    m_obstacles.erase(name);
                }
            }

            std::shared_ptr< std::vector< std::shared_ptr<navigation::Obstacle> > > Settings::obstacles(const std::string& name)
            {
                if (m_obstacles.find(name) == m_obstacles.end())
                {
                    throw UnknownObstacle(fmt::format("There is no set of obstacles with the name: {}", name));
                }
                return m_obstacles.at(name);
            }

            void Settings::saveObstacles(const std::string& name) const
            {
                if (m_obstacles.find(name) != m_obstacles.end())
                {
                    QString save_location = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
                    QString filepath = save_location + QDir::separator() + "obstacles" + QDir::separator() + QString::fromStdString(name) + ".json";
                    QFile f(filepath);
                    if(f.open(QIODevice::WriteOnly))
                    {
                        nlohmann::json j;
                        j["name"] = name;
                        for(std::shared_ptr<navigation::Obstacle> obstacle : *(m_obstacles.at(name)))
                        {
                            nlohmann::json o;
                            std::shared_ptr<navigation::PointObstacle> point = std::dynamic_pointer_cast<navigation::PointObstacle>(obstacle);
                            if(point)
                            {
                                o["point"] = point->centroid();
                            }

                            std::shared_ptr<navigation::LineObstacle> line = std::dynamic_pointer_cast<navigation::LineObstacle>(obstacle);
                            if(line)
                            {
                                o["start"] = line->start();
                                o["end"] = line->end();
                            }

                            std::shared_ptr<navigation::PolygonObstacle> polygon = std::dynamic_pointer_cast<navigation::PolygonObstacle>(obstacle);
                            if(polygon)
                            {
                                o["polygon"] = polygon->polygon();
                            }

                            j["obstacles"].push_back(o);
                        }
                        f.write(j.dump().c_str());
                    }
                }
            }

            void Settings::loadObstacles(const std::string& name)
            {
                QString save_location = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);

                QString obstacles_save_location = save_location + QDir::separator() + "obstacles";
                QDir obstacles_dir(obstacles_save_location);
                if (obstacles_dir.exists())
                {
                    QString filename = QString::fromStdString(name);
                    QFile f(obstacles_save_location + QDir::separator() + filename);
                    if (f.open(QIODevice::ReadOnly))
                    {
                        nlohmann::json j = nlohmann::json::parse(f.readAll());

                        std::vector< std::shared_ptr< navigation::Obstacle > > obstacles;
                        for(const nlohmann::json& v : j["obstacles"])
                        {
                            if(v.find("polygon") != v.end())
                            {
                                obstacles.push_back(std::make_shared<navigation::PolygonObstacle>(v["polygon"].get<geometry::Polygon>()));
                            }
                            else if(v.find("start") != v.end())
                            {
                                obstacles.push_back(std::make_shared<navigation::LineObstacle>(v["start"].get<geometry::Point>(), v["end"].get<geometry::Point>()));
                            }
                            else if(v.find("point") != v.end())
                            {
                                obstacles.push_back(std::make_shared<navigation::PointObstacle>(v["point"].get<geometry::Point>()));
                            }
                            else
                            {
                                throw UnknownObstacle("Unknown obstacle");
                            }
                        }

                        m_obstacles[j["name"]] = std::make_shared< std::vector< std::shared_ptr<navigation::Obstacle> > >(obstacles);

                    }
                }
            }

            std::shared_ptr<navigation::TebConfig> Settings::addConfig(const std::string& name)
            {
                m_config[name] = std::make_shared<navigation::TebConfig>();
                return m_config.at(name);
            }

            void Settings::removeConfig(const std::string& name)
            {
                if(m_config.find(name) != m_config.end())
                {
                    m_config.erase(name);
                }
            }

            std::shared_ptr<navigation::TebConfig> Settings::config(const std::string& name) const
            {
                if(m_config.find(name) == m_config.end())
                {
                    return nullptr;
                }

                return m_config.at(name);
            }

            void Settings::saveConfig(const std::string& name) const
            {
                if (m_config.find(name) != m_config.end())
                {
                    QString save_location = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
                    QString filepath = save_location + QDir::separator() + "config" + QDir::separator() + QString::fromStdString(name) + ".json";
                    QFile f(filepath);
                    if(f.open(QIODevice::WriteOnly))
                    {
                        nlohmann::json j;
                        j["name"] = name;
                        j["config"] = *(m_config.at(name));
                        f.write(j.dump().c_str());
                    }
                }
            }

            void Settings::loadConfig(const std::string& name)
            {
                QString save_location = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);

                QString config_save_location = save_location + QDir::separator() + "config";
                QDir config_dir(config_save_location);
                if (config_dir.exists())
                {
                    QString filename = QString::fromStdString(name);
                    QFile f(config_save_location + QDir::separator() + filename);
                    if (f.open(QIODevice::ReadOnly))
                    {
                        nlohmann::json j = nlohmann::json::parse(f.readAll());
                        m_config[name] = std::make_shared<navigation::TebConfig>(j.get<navigation::TebConfig>());
                    }
                }
            }

            std::vector<std::string> Settings::getRobotNames() const
            {
                std::vector<std::string> names;
                for (auto it : m_robots)
                {
                    names.push_back(it.first);
                }
                return names;
            }

            std::vector<std::string> Settings::getObstaclesNames() const
            {
                std::vector<std::string> names;
                for (auto it : m_obstacles)
                {
                    names.push_back(it.first);
                }
                return names;
            }

            std::vector<std::string> Settings::getConfigNames() const
            {
                std::vector<std::string> names;
                for (auto it : m_config)
                {
                    names.push_back(it.first);
                }
                return names;
            }
        }
    }
}
