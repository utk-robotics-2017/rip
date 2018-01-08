#include "settings_manager.hpp"

#include <QStandardPaths>
#include <QDir>

namespace rip
{
    namespace gui
    {
        namespace pathplanner
        {
            std::shared_ptr<Settings> Settings::m_singleton = nullptr;

            std::shared_ptr<Settings> Settings::getInstance()
            {
                if(!m_singleton)
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
                if(robot_dir.exists())
                {
                    QStringList files = robot_dir.entryList(QDir::Files);
                    for(const QString& filename : files)
                    {
                        QFile f(robot_save_location + QDir::separator() + filename);
                        if(f.open(QIODevice::ReadOnly))
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

                // Load all the course configs
                QString course_save_location = save_location + QDir::separator() + "courses";
                QDir course_dir(course_save_location);
                if(course_dir.exists())
                {
                    QStringList files = course_dir.entryList(QDir::Files);
                    for(const QString& filename : files)
                    {
                        QFile f(course_save_location + QDir::separator() + filename);
                        if(f.open(QIODevice::ReadOnly))
                        {
                            nlohmann::json j = nlohmann::json::parse(f.readAll());
                            m_courses[j["name"]] = std::make_shared<misc::SettingsBase>(j);

                        }
                    }
                }
                else
                {
                    course_dir.mkpath(".");
                }

                // Load all the course configs
                QString path_save_location = save_location + QDir::separator() + "paths";
                QDir path_dir(path_save_location);
                if(path_dir.exists())
                {
                    QStringList files = path_dir.entryList(QDir::Files);
                    for(const QString& filename : files)
                    {
                        QFile f(path_save_location + QDir::separator() + filename);
                        if(f.open(QIODevice::ReadOnly))
                        {
                            nlohmann::json j = nlohmann::json::parse(f.readAll());
                            m_paths[j["name"]] = std::make_shared<misc::SettingsBase>(j);
                        }
                    }
                }
                else
                {
                    path_dir.mkpath(".");
                }

            }

            void Settings::save()
            {
                QString save_location = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
                // Save robot settings
                for(auto iter: m_robots)
                {
                    QString filepath = save_location + QDir::separator() + "robots" + QDir::separator() + QString::fromStdString(iter.first) + ".json";
                    QFile f(filepath);
                    if(f.open(QIODevice::WriteOnly))
                    {
                        nlohmann::json j = *(iter.second);
                        f.write(j.dump().c_str());
                    }
                }

                // Save courses
                for(auto iter: m_courses)
                {
                    QString filepath = save_location + QDir::separator() + "courses" + QDir::separator() + QString::fromStdString(iter.first) + ".json";
                    QFile f(filepath);
                    if(f.open(QIODevice::WriteOnly))
                    {
                        nlohmann::json j = *(iter.second);
                        f.write(j.dump().c_str());
                    }
                }

                // Save paths
                for(auto iter: m_paths)
                {
                    QString filepath = save_location + QDir::separator() + "paths" + QDir::separator() + QString::fromStdString(iter.first) + ".json";
                    QFile f(filepath);
                    if(f.open(QIODevice::WriteOnly))
                    {
                        nlohmann::json j = *(iter.second);
                        f.write(j.dump().c_str());
                    }
                }
            }

            void Settings::addRobot(const std::string& name)
            {
                m_robots[name] = std::make_shared<misc::SettingsBase>(name);
            }

            void Settings::removeRobot(const std::string& name)
            {
                if(m_robots.find(name) != m_robots.end())
                {
                    m_robots.erase(name);
                }
            }

            std::shared_ptr<misc::SettingsBase> Settings::robot(const std::string& name)
            {
                if(m_robots.find(name) == m_robots.end())
                {
                    return nullptr;
                }
                return m_robots[name];
            }

            void Settings::addCourse(const std::string& name)
            {
                m_courses[name] = std::make_shared<misc::SettingsBase>(name);
            }

            void Settings::removeCourse(const std::string& name)
            {
                if(m_courses.find(name) != m_courses.end())
                {
                    m_courses.erase(name);
                }
            }

            std::shared_ptr<misc::SettingsBase> Settings::course(const std::string& name)
            {
                if(m_courses.find(name) == m_courses.end())
                {
                    return nullptr;
                }
                return m_courses[name];
            }

            void Settings::addPath(const std::string &name)
            {
                m_paths[name] = std::make_shared<misc::SettingsBase>(name);
            }

            void Settings::removePath(const std::string& name)
            {
                if(m_paths.find(name) != m_paths.end())
                {
                    m_paths.erase(name);
                }
            }

            std::shared_ptr<misc::SettingsBase> Settings::path(const std::string& name)
            {
                if(m_paths.find(name) == m_paths.end())
                {
                    return nullptr;
                }
                return m_paths[name];
            }

            std::vector<std::string> Settings::getRobotNames()
            {
                std::vector<std::string> names;
                for(auto it : m_robots)
                {
                    names.push_back(it.first);
                }
                return names;
            }

            std::vector<std::string> Settings::getCourseNames()
            {
                std::vector<std::string> names;
                for(auto it : m_courses)
                {
                    names.push_back(it.first);
                }
                return names;
            }

            std::vector<std::string> Settings::getPathNames()
            {
                std::vector<std::string> names;
                for(auto it : m_paths)
                {
                    names.push_back(it.first);
                }
                return names;
            }
        }
    }
}
