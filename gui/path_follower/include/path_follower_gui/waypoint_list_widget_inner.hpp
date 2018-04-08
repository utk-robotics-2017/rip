#ifndef WAYPOINT_LIST_WIDGET_INNER_HPP
#define WAYPOINT_LIST_WIDGET_INNER_HPP

#include <memory>

#include <QListWidget>

#include "path_follower_gui/waypoint_list.hpp"
#include "path_follower_gui/waypoint_widget.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            namespace gui
            {
                class WaypointListWidgetInner : public QListWidget
                {
                    Q_OBJECT
                public:
                    explicit WaypointListWidgetInner(QWidget* parent);

                    void setWaypointList(std::shared_ptr<WaypointList> waypoints);
                    void updateWaypoints();

                protected:
                    void dragEnterEvent(QDragEnterEvent* event) override;
                    void dragMoveEvent(QDragMoveEvent* event) override;
                    void dragLeaveEvent(QDragLeaveEvent* event) override;
                    void dropEvent(QDropEvent* event) override;

                private:
                    std::shared_ptr<WaypointList> m_waypoints;
                    std::map< QListWidgetItem*, std::shared_ptr<WaypointWidget> > m_mapping;
                };
            }
        }
    }
}


#endif // WAYPOINT_LIST_WIDGET_INNER_HPP
