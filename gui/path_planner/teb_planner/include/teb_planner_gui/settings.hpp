#ifndef SETTINGS_MANAGER_HPP
#define SETTINGS_MANAGER_HPP

#include <memory>
#include <vector>
#include <map>

#include <QDir>
#include <QString>

#include <misc/settings_base.hpp>

#include <teb_planner/polygon_robot_footprint_model.hpp>
#include <teb_planner/teb_config.hpp>

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

                ~Settings();

                void load();

                void save() const;

                std::shared_ptr< navigation::PolygonRobotFootprintModel > addRobot(const std::string& name);

                void removeRobot(const std::string& name);

                std::shared_ptr< navigation::PolygonRobotFootprintModel > robot(const std::string& name) const;

                void saveRobot(const std::string& name) const;

                void loadRobot(const std::string& name);

                std::shared_ptr< std::vector< std::shared_ptr<navigation::Obstacle> > > addObstacles(const std::string& name);

                void removeObstacles(const std::string& name);

                std::shared_ptr< std::vector< std::shared_ptr<navigation::Obstacle> > > obstacles(const std::string& name);

                void saveObstacles(const std::string& name) const;

                void loadObstacles(const std::string& name);

                std::shared_ptr< navigation::TebConfig > addConfig(const std::string& name);

                void removeConfig(const std::string& name);

                std::shared_ptr<navigation::TebConfig> config(const std::string& name) const;

                void saveConfig(const std::string& name) const;

                void loadConfig(const std::string& name);

                std::vector<std::string> getRobotNames() const;

                std::vector<std::string> getObstaclesNames() const;

                std::vector<std::string> getConfigNames() const;

            private:
                Settings() = default;

                static std::shared_ptr<Settings> m_singleton;

                std::map< std::string, std::shared_ptr<navigation::PolygonRobotFootprintModel> > m_robots;
                std::map< std::string, std::shared_ptr<navigation::TebConfig> > m_config;
                std::map< std::string, std::shared_ptr< std::vector< std::shared_ptr<navigation::Obstacle> > > > m_obstacles;
            };
        }
    }
}

#endif
