#ifndef WAYPOINT_LIST_WIDGET_OUTER_HPP
#define WAYPOINT_LIST_WIDGET_OUTER_HPP

#include <memory>

#include <QWidget>

#include "path_follower_gui/waypoint_list.hpp"

namespace Ui
{
    class WaypointListWidgetOuter;
}

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            namespace gui
            {
                class WaypointListWidgetOuter : public QWidget
                {
                    Q_OBJECT
                public:
                    explicit WaypointListWidgetOuter(QWidget* parent = nullptr);
                    ~WaypointListWidgetOuter();

                public slots:
                    void waypointsOptionsChanged();
                    void updateWaypoints();

                private slots:
                    void add();
                    void remove();
                    void addWaypoint();

                private:
                    Ui::WaypointListWidgetOuter* m_ui;
                    std::shared_ptr<WaypointList> m_waypoints;
                };
            }
        }
    }
}

#endif // WAYPOINT_LIST_WIDGET_OUTER_HPP
