#include "path_follower_gui/waypoint_list_widget_inner.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            namespace gui
            {

                WaypointListWidgetInner::WaypointListWidgetInner(QWidget* parent)
                    : QListWidget(parent)
                {
                    setAcceptDrops(true);
                    setDragEnabled(true);

                    setSelectionMode(QAbstractItemView::SingleSelection);
                    setDropIndicatorShown(true);
                    setDragDropMode(QAbstractItemView::InternalMove);
                }

                void WaypointListWidgetInner::setWaypointList(std::shared_ptr<WaypointList> waypoints)
                {
                    clear();
                    m_mapping.clear();

                    m_waypoints = waypoints;

                    if(m_waypoints)
                    {
                        int index = 0;
                        for(const Waypoint& waypoint : *m_waypoints)
                        {
                            QListWidgetItem* lwi = new QListWidgetItem;
                            std::shared_ptr<WaypointWidget> ww = std::make_shared<WaypointWidget>(waypoint);
                            ww->setIndex(index++);
                            lwi->setSizeHint(ww->sizeHint());
                            ww->show();
                            addItem(lwi);
                            setItemWidget(lwi, ww.get());
                            m_mapping[lwi] = ww;
                        }
                        update();
                    }
                }

                void WaypointListWidgetInner::updateWaypoints()
                {
                    clear(); // todo: check if this deletes the QListWidgetItem* s
                    m_mapping.clear();

                    if(m_waypoints)
                    {
                        int index = 0;
                        for(const Waypoint& waypoint : *m_waypoints)
                        {
                            QListWidgetItem* lwi = new QListWidgetItem;
                            std::shared_ptr<WaypointWidget> ww = std::make_shared<WaypointWidget>(waypoint);
                            ww->setIndex(index++);
                            lwi->setSizeHint(ww->sizeHint());
                            ww->show();
                            addItem(lwi);
                            setItemWidget(lwi, ww.get());
                            m_mapping[lwi] = ww;
                        }
                        update();
                    }
                }

                void WaypointListWidgetInner::dragEnterEvent(QDragEnterEvent* event)
                {

                }

                void WaypointListWidgetInner::dragMoveEvent(QDragMoveEvent* event)
                {

                }

                void WaypointListWidgetInner::dragLeaveEvent(QDragLeaveEvent* event)
                {

                }

                void WaypointListWidgetInner::dropEvent(QDropEvent* event)
                {

                }

            }
        }
    }
}
