#include <teb_planner_gui/settings.hpp>

#include <QStandardPaths>
#include <QDir>

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

            void Settings::load()
            {
                QString save_location = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);

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
                            m_robots[j["name"]] = std::make_shared<misc::SettingsBase>(j);
                        }
                    }
                }
                else
                {
                    robot_dir.mkpath(".");
                }

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
                            m_obstacles[j["name"]] = std::make_shared<misc::SettingsBase>(j);

                        }
                    }
                }
                else
                {
                    obstacles_dir.mkpath(".");
                }

                // Load all the course configs
                QString trajectory_save_location = save_location + QDir::separator() + "trajectories";
                QDir trajectory_dir(trajectory_save_location);
                if (trajectory_dir.exists())
                {
                    QStringList files = trajectory_dir.entryList(QDir::Files);
                    for (const QString& filename : files)
                    {
                        QFile f(trajectory_save_location + QDir::separator() + filename);
                        if (f.open(QIODevice::ReadOnly))
                        {
                            nlohmann::json j = nlohmann::json::parse(f.readAll());
                            m_trajectories[j["name"]] = std::make_shared<misc::SettingsBase>(j);
                        }
                    }
                }
                else
                {
                    trajectory_dir.mkpath(".");
                }

            }

            void Settings::save() const
            {
                QString save_location = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
                // Save robot settings
                for (auto iter : m_robots)
                {
                    QString filepath = save_location + QDir::separator() + "robots" + QDir::separator() + QString::fromStdString(iter.first) + ".json";
                    QFile f(filepath);
                    if (f.open(QIODevice::WriteOnly))
                    {
                        nlohmann::json j = *(iter.second);
                        f.write(j.dump().c_str());
                    }
                }

                // Save courses
                for (auto iter : m_obstacles)
                {
                    QString filepath = save_location + QDir::separator() + "obstacles" + QDir::separator() + QString::fromStdString(iter.first) + ".json";
                    QFile f(filepath);
                    if (f.open(QIODevice::WriteOnly))
                    {
                        nlohmann::json j = *(iter.second);
                        f.write(j.dump().c_str());
                    }
                }

                // Save paths
                for (auto iter : m_trajectories)
                {
                    QString filepath = save_location + QDir::separator() + "trajectories" + QDir::separator() + QString::fromStdString(iter.first) + ".json";
                    QFile f(filepath);
                    if (f.open(QIODevice::WriteOnly))
                    {
                        nlohmann::json j = *(iter.second);
                        f.write(j.dump().c_str());
                    }
                }
            }

            std::shared_ptr<misc::SettingsBase> Settings::addRobot(const std::string& name)
            {
                m_robots[name] = std::make_shared<misc::SettingsBase>(name);
                return m_robots[name];
            }

            void Settings::removeRobot(const std::string& name)
            {
                if (m_robots.find(name) != m_robots.end())
                {
                    m_robots.erase(name);
                }
            }

            std::shared_ptr<misc::SettingsBase> Settings::robot(const std::string& name) const
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
                        nlohmann::json j = *m_robots.at(name);
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
                        m_robots[name] = std::make_shared<misc::SettingsBase>(j);
                    }
                }
            }

            std::shared_ptr<misc::SettingsBase> Settings::addObstacles(const std::string& name)
            {
                m_obstacles[name] = std::make_shared<misc::SettingsBase>(name);
                return m_obstacles[name];
            }

            void Settings::removeObstacles(const std::string& name)
            {
                if (m_obstacles.find(name) != m_obstacles.end())
                {
                    m_obstacles.erase(name);
                }
            }

            std::shared_ptr<misc::SettingsBase> Settings::obstacles(const std::string& name) const
            {
                if (m_obstacles.find(name) == m_obstacles.end())
                {
                    return nullptr;
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
                    if (f.open(QIODevice::WriteOnly))
                    {
                        nlohmann::json j = *m_obstacles.at(name);
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
                        m_obstacles[name] = std::make_shared<misc::SettingsBase>(j);

                    }
                }
            }

            std::shared_ptr<misc::SettingsBase> Settings::addTrajectory(const std::string& name)
            {
                m_trajectories[name] = std::make_shared<misc::SettingsBase>(name);
                return m_trajectories.at(name);
            }

            void Settings::removeTrajectory(const std::string& name)
            {
                if (m_trajectories.find(name) != m_trajectories.end())
                {
                    m_trajectories.erase(name);
                }
            }

            std::shared_ptr<misc::SettingsBase> Settings::trajectory(const std::string& name) const
            {
                if (m_trajectories.find(name) == m_trajectories.end())
                {
                    return nullptr;
                }
                return m_trajectories.at(name);
            }

            void Settings::saveTrajectory(const std::string& name) const
            {
                if (m_trajectories.find(name) != m_trajectories.end())
                {
                    QString save_location = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
                    QString filepath = save_location + QDir::separator() + "trajectories" + QDir::separator() + QString::fromStdString(name) + ".json";
                    QFile f(filepath);
                    if (f.open(QIODevice::WriteOnly))
                    {
                        nlohmann::json j = *m_trajectories.at(name);
                        f.write(j.dump().c_str());
                    }
                }
            }

            void Settings::loadTrajectory(const std::string& name)
            {
                QString save_location = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);

                QString trajectory_save_location = save_location + QDir::separator() + "trajectories";
                QDir trajectory_dir(trajectory_save_location);
                if (trajectory_dir.exists())
                {
                    QString filename = QString::fromStdString(name);
                    QFile f(trajectory_save_location + QDir::separator() + filename);
                    if (f.open(QIODevice::ReadOnly))
                    {
                        nlohmann::json j = nlohmann::json::parse(f.readAll());
                        m_trajectories[name] = std::make_shared<misc::SettingsBase>(j);
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

            std::vector<std::string> Settings::getTrajectoryNames() const
            {
                std::vector<std::string> names;
                for (auto it : m_trajectories)
                {
                    names.push_back(it.first);
                }
                return names;
            }
        }
    }
}
