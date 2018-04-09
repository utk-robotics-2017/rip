#ifndef STORAGE_HPP
#define STORAGE_HPP

#include <memory>

#include "robot.hpp"
#include "waypoint_list.hpp"
#include "world.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            namespace gui
            {
                class Storage : public QObject
                {
                    Q_OBJECT
                public:
                    static std::shared_ptr<Storage> getInstance();

                    void load();
                    void save();

                    std::shared_ptr<Robot> loadRobot(const QString& filename);
                    std::shared_ptr<WaypointList> loadWaypoints(const QString& filename);
                    std::shared_ptr<World> loadWorld(const QString& filename);

                    std::shared_ptr<Robot> loadRobot(const std::string& filename);
                    std::shared_ptr<WaypointList> loadWaypoints(const std::string& filename);
                    std::shared_ptr<World> loadWorld(const std::string& filename);

                    std::shared_ptr<Robot> addRobot(const std::string& name);
                    std::shared_ptr<WaypointList> addWaypoints(const std::string& name);
                    std::shared_ptr<World> addWorld(const std::string& name);

                    bool removeRobot(const std::string& name);
                    bool removeWaypoints(const std::string& name);
                    bool removeWorld(const std::string& name);

                    std::shared_ptr<World> world(const std::string& name) const;
                    std::shared_ptr<WaypointList> waypoints(const std::string& name) const;
                    std::shared_ptr<Robot> robot(const std::string& name) const;

                    std::vector<std::string> worldNames() const;
                    std::vector<std::string> waypointNames() const;
                    std::vector<std::string> robotNames() const;

                    std::shared_ptr<World> selectedWorld() const;
                    std::shared_ptr<Robot> selectedRobot() const;
                    std::shared_ptr<WaypointList> selectedWaypoints() const;

                    std::string selectedWorldName() const;
                    std::string selectedRobotName() const;
                    std::string selectedWaypointsName() const;

                public slots:
                    void selectWorld(const QString& name);
                    void selectRobot(const QString& name);
                    void selectWaypoints(const QString& name);
                    void waypointsUpdated();

                signals:
                    void worldOptionsChanged();
                    void waypointsOptionsChanged();
                    void robotOptionsChanged();

                    void selectedWorldChanged();
                    void selectedRobotChanged();
                    void selectedWaypointsChanged();

                    void waypointsUpdate();

                private:
                    Storage() = default;
                    static std::shared_ptr<Storage> m_singleton;

                    std::map<std::string, std::shared_ptr<World> > m_worlds;
                    std::map<std::string, std::shared_ptr<WaypointList> > m_waypoints;
                    std::map<std::string, std::shared_ptr<Robot> > m_robots;

                    std::string m_selected_world;
                    std::string m_selected_robot;
                    std::string m_selected_waypoints;
                };
            }
        }
    }
}

#endif // STORAGE_HPP
