#include "path_follower_gui/waypoint_list.hpp"
#include <QFile>
namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            namespace gui
            {
                std::string WaypointList::name() const
                {
                    return m_name;
                }

                void WaypointList::setName(const std::string& name)
                {
                    m_name = name;
                }

                std::vector<std::shared_ptr<Waypoint>> WaypointList::waypoints() const
                {
                    return m_waypoints;
                }

                std::shared_ptr<Waypoint> WaypointList::get(int index) const
                {
                    assert(index >= 0 && index < m_waypoints.size());
                    return m_waypoints[index];
                }

                void WaypointList::setPoint(int index, const geometry::Point& point)
                {
                    m_waypoints[index]->setX(point.x());
                    m_waypoints[index]->setY(point.y());
                }

                void WaypointList::addWaypoint(const Waypoint& waypoint)
                {
                    m_waypoints.push_back(std::make_shared<Waypoint>(waypoint));
                }

                void WaypointList::insertWaypoint(int index, const Waypoint& waypoint)
                {
                    // todo: assert?
                    m_waypoints.insert(m_waypoints.begin() +index, std::make_shared<Waypoint>(waypoint));
                }

                void WaypointList::removeWaypoint(int index)
                {
                    assert(index >= 0 && index < m_waypoints.size());
                    m_waypoints.erase(m_waypoints.begin() + index);
                }

                int WaypointList::size() const
                {
                    return m_waypoints.size();
                }

                void WaypointList::save(const QString& filepath) const
                {
                    QFile f(filepath);
                    if (f.open(QIODevice::WriteOnly))
                    {
                        nlohmann::json j;
                        j["name"] = m_name;
                        std::vector<Waypoint> waypoints;
                        for(std::shared_ptr<Waypoint> waypoint : m_waypoints)
                        {
                            waypoints.push_back(*waypoint);
                        }
                        j["waypoints"] = waypoints;
                        f.write(j.dump(4).c_str());
                    }
                }

                void WaypointList::draw(QPainter& painter, float circle_diameter) const
                {
                    painter.setBrush(Qt::blue);

                    for(std::shared_ptr<Waypoint> waypoint : m_waypoints)
                    {
                        geometry::Point center = waypoint->position();
                        // Draw Circle for Waypoint
                        painter.drawEllipse(center.x()() - circle_diameter / 2.0, center.y()() - circle_diameter / 2.0, circle_diameter, circle_diameter);
                    }
                }

                std::vector<std::shared_ptr<Waypoint>>::iterator WaypointList::begin()
                {
                    return m_waypoints.begin();
                }

                std::vector<std::shared_ptr<Waypoint>>::iterator WaypointList::end()
                {
                    return m_waypoints.end();
                }

                std::vector<std::shared_ptr<Waypoint>>::const_iterator WaypointList::begin() const
                {
                    return m_waypoints.cbegin();
                }

                std::vector<std::shared_ptr<Waypoint>>::const_iterator WaypointList::end() const
                {
                    return m_waypoints.cend();
                }

               std::shared_ptr<WaypointList> WaypointList::load(const QString& filepath)
               {
                   QFile f(filepath);
                   if (f.open(QIODevice::ReadOnly))
                   {
                       nlohmann::json j = nlohmann::json::parse(f.readAll());
                       std::shared_ptr<WaypointList> waypoints = std::make_shared<WaypointList>();
                       waypoints->m_name = j["name"];
                       for(const nlohmann::json& w : j["waypoints"])
                       {
                           waypoints->m_waypoints.push_back(std::make_shared<Waypoint>(w.get<Waypoint>()));
                       }
                       return waypoints;
                   }
               }
            }
        }
    }
}
