#ifndef WAYPOINT_LIST_WIDGET_ITEM_HPP
#define WAYPOINT_LIST_WIDGET_ITEM_HPP

#include <QListWidgetItem>

#include <spline_planner/waypoint.hpp>

namespace Ui
{
    class WaypointListWidgetItem;
}


namespace rip
{
    namespace gui
    {
        namespace splineplanner
        {
            class WaypointListWidgetItem : public QWidget
            {
                Q_OBJECT
            public:
                explicit WaypointListWidgetItem(QWidget* parent = nullptr);

                ~WaypointListWidgetItem();

                double x() const;
                void setX(double x);

                double y() const;
                void setY(double y);

                double angle() const;
                void setAngle(double angle);

                void setWaypoint(const navigation::Waypoint& waypoint);
                navigation::Waypoint waypoint() const;

                bool valid() const;

                virtual QSize minimumSizeHint() const;
                virtual QSize sizeHint() const;

            signals:
                void modified();

            public slots:
                void updatedText();

            private:
                Ui::WaypointListWidgetItem* m_ui;
            };
        }
    }
}

#endif // WAYPOINT_LIST_WIDGET_ITEM_HPP
