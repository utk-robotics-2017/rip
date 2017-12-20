#include "settings_manager.hpp"

#include <QStandardPaths>
#include <QDir>

namespace rip
{
    namespace gui
    {
        namespace pathplanner
        {
            std::shared_ptr<SettingsManager> SettingsManager::m_singleton = nullptr;

            std::shared_ptr<SettingsManager> SettingsManager::getInstance()
            {
                if(!m_singleton)
                {
                    m_singleton = std::shared_ptr<SettingsManager>(new SettingsManager);
                }
                return m_singleton;
            }

            void SettingsManager::load()
            {
                std::string save_location = QStandardPaths::writeableLocation(QStandardPaths::AppDataLocation).toStdString();

                // Load all the robot configs
                std::string robot_save_location = save_location + QDir::separator().toLatin1() + "robots";
                FileHandle robot_dir = fs::open(robot_save_location);
                if(robot_dir.isDirectory())
                {
                    std::vector<std::string> files = robot_dir.listFiles();
                    for(const std::string& filename : files)
                    {
                        FileHandle fh = fs::open(filename);
                        std::unique_ptr<std::istream> input = fh.createInputStream();
                        nlohamann::json j;
                        (*input) >> j;
                        m_robots[j["name"]] = std::make_shared<SettingsBase>(j);
                    }
                }
                else
                {
                    robot_dir.createDirectory();
                }

                // Load all the course configs
                std::string course_save_location = save_location + QDir::separator().toLatin1() + "course";
                FileHandle course_dir = fs::open(course_save_location);
                if(course_dir.isDirectory())
                {
                    std::vector<std::string> files = course_dir.listFiles();
                    for(const std::string& filename : files)
                    {
                        FileHandle fh = fs::open(filename);
                        std::unique_ptr<std::istream> input = fh.createInputStream();
                        nlohamann::json j;
                        (*input) >> j;
                        m_courses[j["name"]] = std::make_shared<SettingsBase>(j);
                    }
                }
                else
                {
                    course_dir.createDirectory();
                }

            }

            void SettingsManager::save()
            {
                std::string save_location = QStandardPaths::writeableLocation(QStandardPaths::AppDataLocation).toStdString();
                for(auto iter: m_robots)
                {
                    std::string filepath = save_location + QDir::separator().toLatin1() + "robots" + QDir::separator().toLatin1() + iter.first + ".json";
                    FileHandle fh = fs::open(filename);
                    std::unique_ptr<std::ostream> out = fh.createOutputStream();
                    (*out) << *(iter.second);
                }
                for(auto iter: m_robots)
                {
                    std::string filepath = save_location + QDir::separator().toLatin1() + "robots" + QDir::separator().toLatin1() + iter.first + ".json";
                    FileHandle fh = fs::open(filename);
                    std::unique_ptr<std::ostream> out = fh.createOutputStream();
                    (*out) << *(iter.second);
                }
            }

            void SettingsManager::addRobot(const std::string& name)
            {
                m_robots[name] = std::make_shared<SettingsBase>(name);
            }

            std::shared_ptr<SettingsBase> SettingsManager::robot(const std::string& name) const
            {
                if(m_robots.find(name) == m_robots.end())
                {
                    return nullptr;
                }
                return m_robots[name];
            }

            void SettingsManager::addCourse(const std::string& name)
            {
                m_courses[name] = std::make_shared<SettingsBase>(name);
            }

            std::shared_ptr<SettingsBase> SettingsManager::course(const std::string& name) const
            {
                if(m_courses.find(name) == m_courses.end())
                {
                    return nullptr;
                }
                return m_courses[name];
            }

            std::vector<std::string> SettingsManager::getRobotNames()
            {
                std::vector<std::string> names;
                for(auto it : m_robots)
                {
                    names.push_back(it.first);
                }
                return names;
            }

            std::vector<std::string> SettingsManager::getCourseNames()
            {
                std::vector<std::string> names;
                for(auto it : m_courses)
                {
                    names.push_back(it.first);
                }
                return names;
            }
        }
    }
}
