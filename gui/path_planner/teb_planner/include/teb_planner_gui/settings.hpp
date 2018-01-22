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
        namespace tebplanner
        {
            class Settings
            {
            public:
                static std::shared_ptr<Settings> getInstance();

                void load();

                void save() const;

                std::shared_ptr<misc::SettingsBase> addRobot(const std::string& name);

                void removeRobot(const std::string& name);

                std::shared_ptr<misc::SettingsBase> robot(const std::string& name) const;

                void saveRobot(const std::string& name) const;

                void loadRobot(const std::string& name);

                std::shared_ptr<misc::SettingsBase> addObstacles(const std::string& name);

                void removeObstacles(const std::string& name);

                std::shared_ptr<misc::SettingsBase> obstacles(const std::string& name) const;

                void saveObstacles(const std::string& name) const;

                void loadObstacles(const std::string& name);

                std::shared_ptr<misc::SettingsBase> addTrajectory(const std::string& name);

                void removeTrajectory(const std::string& name);

                std::shared_ptr<misc::SettingsBase> trajectory(const std::string& name) const;

                void saveTrajectory(const std::string& name) const;

                void loadTrajectory(const std::string& name);

                std::vector<std::string> getRobotNames() const;

                std::vector<std::string> getObstaclesNames() const;

                std::vector<std::string> getTrajectoryNames() const;

            private:
                Settings() = default;

                static std::shared_ptr<Settings> m_singleton;

                std::map< std::string, std::shared_ptr<misc::SettingsBase> > m_robots;
                std::map< std::string, std::shared_ptr<misc::SettingsBase> > m_obstacles;
                std::map< std::string, std::shared_ptr<misc::SettingsBase> > m_trajectories;
            };
        }
    }
}

#endif
