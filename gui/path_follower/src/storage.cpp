#include "path_follower_gui/storage.hpp"

#include <QStandardPaths>
#include <QDir>

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            namespace gui
            {
                std::shared_ptr<Storage> Storage::m_singleton = nullptr;

                std::shared_ptr<Storage> Storage::getInstance()
                {
                    if (!m_singleton)
                    {
                        m_singleton = std::shared_ptr<Storage>(new Storage);
                    }
                    return m_singleton;
                }

                void Storage::load()
                {
                    QString save_location = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
                    QDir sl(save_location);
                    if (!sl.exists())
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
                                loadRobot(robot_save_location + QDir::separator() + filename);
                            }
                        }
                        else
                        {
                            robot_dir.mkpath(".");
                        }
                    }

                    {
                        // Load all the worlds
                        QString worlds_save_location = save_location + QDir::separator() + "worlds";
                        QDir worlds_dir(worlds_save_location);
                        if (worlds_dir.exists())
                        {
                            QStringList files = worlds_dir.entryList(QDir::Files);
                            for (const QString& filename : files)
                            {
                                loadWorld(worlds_save_location + QDir::separator() +filename);
                            }
                        }
                        else
                        {
                            worlds_dir.mkpath(".");
                        }
                    }

                    {
                        // Load Waypoints
                        QString waypoints_save_location = save_location + QDir::separator() + "waypoints";
                        QDir waypoints_dir(waypoints_save_location);
                        if (waypoints_dir.exists())
                        {
                            QStringList files = waypoints_dir.entryList(QDir::Files);
                            for (const QString& filename : files)
                            {
                                loadWaypoints(waypoints_save_location + QDir::separator() + filename);
                            }
                        }
                        else
                        {
                            waypoints_dir.mkpath(".");
                        }
                    }
                }

                void Storage::save()
                {
                    QString save_location = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);

                    if (m_robots.size())
                    {
                        // Save robot footprint
                        for (auto iter : m_robots)
                        {
                            QString filepath = save_location + QDir::separator() + "robots" + QDir::separator() + QString::fromStdString(iter.first) + ".json";
                            iter.second->save(filepath);
                        }
                    }

                    if (m_worlds.size())
                    {
                        // Save Obstacles
                        for (auto iter : m_worlds)
                        {
                            QString filepath = save_location + QDir::separator() + "worlds" + QDir::separator() + QString::fromStdString(iter.first) + ".json";
                            iter.second->save(filepath);
                        }
                    }

                    if (m_waypoints.size())
                    {
                        // Save waypoints
                        for (auto iter : m_waypoints)
                        {
                            QString filepath = save_location + QDir::separator() + "waypoints" + QDir::separator() + QString::fromStdString(iter.first) + ".json";
                            iter.second->save(filepath);
                        }
                    }
                }

                std::shared_ptr<Robot> Storage::loadRobot(const QString &filepath)
                {
                    std::shared_ptr<Robot> robot = Robot::load(filepath);
                    m_robots[robot->name()] = robot;
                    emit robotOptionsChanged();
                    return robot;
                }

                std::shared_ptr<Robot> Storage::loadRobot(const std::string& filepath)
                {
                    return loadRobot(QString::fromStdString(filepath));
                }

                std::shared_ptr<WaypointList> Storage::loadWaypoints(const QString &filepath)
                {
                    std::shared_ptr<WaypointList> waypoints = WaypointList::load(filepath);
                    m_waypoints[waypoints->name()] = waypoints;
                    emit waypointsOptionsChanged();
                    return waypoints;
                }

                std::shared_ptr<WaypointList> Storage::loadWaypoints(const std::string& filepath)
                {
                    return loadWaypoints(QString::fromStdString(filepath));
                }

                std::shared_ptr<World> Storage::loadWorld(const QString &filepath)
                {
                    std::shared_ptr<World> world = World::load(filepath);
                    m_worlds[world->name()] = world;
                    emit worldOptionsChanged();
                    return world;
                }



                std::shared_ptr<World> Storage::loadWorld(const std::string& filepath)
                {
                    return loadWorld(QString::fromStdString(filepath));
                }

                std::shared_ptr<Robot> Storage::addRobot(const std::string& name)
                {
                    std::shared_ptr<Robot> robot = std::make_shared<Robot>();
                    robot->setName(name);
                    m_robots[name] = robot;
                    m_selected_robot = name;
                    emit robotOptionsChanged();
                    return robot;
                }

                std::shared_ptr<WaypointList> Storage::addWaypoints(const std::string& name)
                {
                    std::shared_ptr<WaypointList> waypoints = std::make_shared<WaypointList>();
                    waypoints->setName(name);
                    m_waypoints[name] = waypoints;
                    m_selected_waypoints = name;
                    emit waypointsOptionsChanged();
                    return waypoints;
                }

                std::shared_ptr<World> Storage::addWorld(const std::string& name)
                {
                    std::shared_ptr<World> world = std::make_shared<World>();
                    world->setName(name);
                    m_worlds[name] = world;

                    emit worldOptionsChanged();
                    return world;
                }

                bool Storage::removeRobot(const std::string& name)
                {
                    if(m_robots.find(name) != m_robots.end())
                    {
                        m_robots.erase(name);
                        if(m_selected_robot == name)
                        {
                            m_selected_robot = "";
                        }
                        emit robotOptionsChanged();
                        return true;
                    }
                    return false;
                }

                bool Storage::removeWaypoints(const std::string& name)
                {
                    if(m_waypoints.find(name) != m_waypoints.end())
                    {
                        m_waypoints.erase(name);
                        if(m_selected_waypoints == name)
                        {
                            m_selected_waypoints = "";
                        }
                        emit waypointsOptionsChanged();
                        return true;
                    }
                    return false;
                }

                bool Storage::removeWorld(const std::string& name)
                {
                    if(m_worlds.find(name) != m_worlds.end())
                    {
                        m_worlds.erase(name);
                        emit worldOptionsChanged();
                        if(m_selected_world == name)
                        {
                            m_selected_world = "";
                        }
                        return true;
                    }
                    return false;
                }

                std::shared_ptr<World> Storage::world(const std::string& name) const
                {
                    if(m_worlds.find(name) == m_worlds.end())
                    {
                        return nullptr;
                    }
                    return m_worlds.at(name);
                }

                std::shared_ptr<WaypointList> Storage::waypoints(const std::string& name) const
                {
                    if(m_waypoints.find(name) == m_waypoints.end())
                    {
                        return nullptr;
                    }
                    return m_waypoints.at(name);
                }

                std::shared_ptr<Robot> Storage::robot(const std::string& name) const
                {
                    if(m_robots.find(name) == m_robots.end())
                    {
                        return nullptr;
                    }
                    return m_robots.at(name);
                }

                std::vector<std::string> Storage::worldNames() const
                {
                    std::vector<std::string> keys;
                    std::transform(m_worlds.begin(), m_worlds.end(), std::back_inserter(keys), [](const std::map< std::string, std::shared_ptr<World> >::value_type & pair) {return pair.first;});
                    return keys;
                }

                std::vector<std::string> Storage::waypointNames() const
                {
                    std::vector<std::string> keys;
                    std::transform(m_waypoints.begin(), m_waypoints.end(), std::back_inserter(keys), [](const std::map< std::string, std::shared_ptr<WaypointList> >::value_type & pair) {return pair.first;});
                    return keys;
                }

                std::vector<std::string> Storage::robotNames() const
                {
                    std::vector<std::string> keys;
                    std::transform(m_robots.begin(), m_robots.end(), std::back_inserter(keys), [](const std::map< std::string, std::shared_ptr<Robot> >::value_type & pair) {return pair.first;});
                    return keys;
                }

                std::shared_ptr<World> Storage::selectedWorld() const
                {
                    if(m_worlds.find(m_selected_world) == m_worlds.end())
                    {
                        return nullptr;
                    }
                    return m_worlds.at(m_selected_world);
                }

                std::shared_ptr<Robot> Storage::selectedRobot() const
                {
                    if(m_robots.find(m_selected_robot) == m_robots.end())
                    {
                        return nullptr;
                    }
                    return m_robots.at(m_selected_robot);
                }

                std::shared_ptr<WaypointList> Storage::selectedWaypoints() const
                {
                    if(m_waypoints.find(m_selected_waypoints) == m_waypoints.end())
                    {
                        return nullptr;
                    }
                    return m_waypoints.at(m_selected_waypoints);
                }

                std::string Storage::selectedWorldName() const
                {
                    return m_selected_world;
                }

                std::string Storage::selectedRobotName() const
                {
                    return m_selected_robot;
                }

                std::string Storage::selectedWaypointsName() const
                {
                    return m_selected_waypoints;
                }

                void Storage::selectWorld(const QString& name)
                {
                    m_selected_world = name.toStdString();
                    emit selectedWorldChanged();
                }

                void Storage::selectRobot(const QString& name)
                {
                    m_selected_robot = name.toStdString();
                    emit selectedRobotChanged();
                }

                void Storage::selectWaypoints(const QString& name)
                {
                    m_selected_waypoints = name.toStdString();
                    emit selectedWaypointsChanged();
                }

                void Storage::waypointsUpdated()
                {
                    emit waypointsUpdate();
                }
            }
        }
    }
}
