#include "path_follower_gui/waypoint_list.hpp"

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

                std::vector<Waypoint> WaypointList::waypoints() const
                {
                    return m_waypoints;
                }

                void WaypointList::addWaypoint(const Waypoint& waypoint)
                {
                    m_waypoints.push_back(waypoint);
                }

                void WaypointList::insertWaypoint(int index, const Waypoint& waypoint)
                {
                    // todo: assert?
                    m_waypoints.insert(m_waypoints.begin() +index, waypoint);
                }

                void WaypointList::removeWaypoint(int index)
                {
                    assert(index >= 0 && index < m_waypoints.size());
                    m_waypoints.erase(m_waypoints.begin() + index);
                }

                void WaypointList::save(const QString& filepath) const
                {
                    // todo
                }

                void WaypointList::draw(QPainter& painter, float circle_radius) const
                {
                    painter.setBrush(Qt::blue);

                    for(const Waypoint& waypoint : m_waypoints)
                    {
                        geometry::Point center = waypoint.position();
                        // Draw Circle for Waypoint
                        painter.drawEllipse(center.x()(), center.y()(), circle_radius, circle_radius);
                    }
                }

                std::vector<Waypoint>::iterator WaypointList::begin()
                {
                    return m_waypoints.begin();
                }

                std::vector<Waypoint>::iterator WaypointList::end()
                {
                    return m_waypoints.end();
                }

                std::vector<Waypoint>::const_iterator WaypointList::begin() const
                {
                    return m_waypoints.cbegin();
                }

                std::vector<Waypoint>::const_iterator WaypointList::end() const
                {
                    return m_waypoints.cend();
                }

               std::shared_ptr<WaypointList> WaypointList::load(const QString& filepath)
               {
                   // todo
                    return nullptr;
               }
            }
        }
    }
}
