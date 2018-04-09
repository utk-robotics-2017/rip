#ifndef WAYPOINT_LIST_HPP
#define WAYPOINT_LIST_HPP

#include <vector>

#include <QString>
#include <QPainter>

#include "path_follower/waypoint.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            namespace gui
            {
                class WaypointList
                {
                public:
                    WaypointList() = default;

                    std::string name() const;
                    void setName(const std::string& name);

                    std::vector< std::shared_ptr<Waypoint> > waypoints() const;
                    std::shared_ptr<Waypoint> get(int index) const;
                    void setPoint(int index, const geometry::Point& point);
                    void addWaypoint(const Waypoint& waypoint);
                    void insertWaypoint(int index, const Waypoint& waypoint);
                    void removeWaypoint(int index);
                    int size() const;

                    void save(const QString& filepath) const;
                    void draw(QPainter& painter, float circle_radius) const;

                    std::vector< std::shared_ptr<Waypoint> >::iterator begin();
                    std::vector< std::shared_ptr<Waypoint> >::iterator end();
                    std::vector< std::shared_ptr<Waypoint> >::const_iterator begin() const;
                    std::vector< std::shared_ptr<Waypoint> >::const_iterator end() const;

                   static std::shared_ptr<WaypointList> load(const QString& filepath);
                private:
                    std::string m_name;
                    std::vector< std::shared_ptr<Waypoint> > m_waypoints;
                };
            }
        }
    }
}


#endif // WAYOINT_LIST_HPP
