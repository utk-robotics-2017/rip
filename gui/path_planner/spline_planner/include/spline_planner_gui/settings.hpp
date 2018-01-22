#ifndef SETTINGS_MANAGER_HPP
#define SETTINGS_MANAGER_HPP

#include <memory>
#include <vector>
#include <map>

#include <QDir>
#include <QString>

#include <misc/settings_base.hpp>

namespace rip
{
    namespace gui
    {
        namespace splineplanner
        {
            class Settings
            {
            public:
                static std::shared_ptr<Settings> getInstance();

                void load();

                void save();

                void addRobot(const std::string& name);

                void removeRobot(const std::string& name);

                std::shared_ptr<misc::SettingsBase> robot(const std::string& name);

                void addCourse(const std::string& name);

                void removeCourse(const std::string& name);

                std::shared_ptr<misc::SettingsBase> course(const std::string& name);

                void addPath(const std::string& name);

                void removePath(const std::string& name);

                std::shared_ptr<misc::SettingsBase> path(const std::string& name);

                std::vector<std::string> getRobotNames();

                std::vector<std::string> getCourseNames();

                std::vector<std::string> getPathNames();

            private:
                Settings() = default;

                static std::shared_ptr<Settings> m_singleton;

                std::map< std::string, std::shared_ptr<misc::SettingsBase> > m_robots;
                std::map< std::string, std::shared_ptr<misc::SettingsBase> > m_courses;
                std::map< std::string, std::shared_ptr<misc::SettingsBase> > m_paths;
            };
        }
    }
}

#endif
