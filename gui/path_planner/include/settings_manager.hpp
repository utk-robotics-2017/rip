#ifndef SETTINGS_MANAGER_HPP
#define SETTINGS_MANAGER_HPP

#include <memory>
#include <vector>
#include <map>

#include <QDir>
#include <QString>

#include "settings_base.hpp"

namespace rip
{
    namespace gui
    {
        namespace pathplanner
        {
            class SettingsManager
            {
            public:
                static std::shared_ptr<SettingsManager> getInstance();

                void load();

                void save();

                void addRobot(const std::string& name);

                std::shared_ptr<SettingsBase> robot(const std::string& name) const;

                void addCourse(const std::string& name);

                std::shared_ptr<SettingsBase> course(const std::string& name) const;

                std::vector<std::string> getRobotNames();

                std::vector<std::string> getCourseNames();

            private:
                SettingsManager() = default;

                static std::shared_ptr<SettingsManager> m_singleton;

                std::map< std::string, std::shared_ptr<SettingsBase> > m_robots;
                std::map< std::string, std::shared_ptr<SettingsBase> > m_courses;
            };
        }
    }
}

#endif
