#ifndef WAYPOINT_WIDGET_HPP
#define WAYPOINT_WIDGET_HPP

#include <units/units.hpp>
#include <path_follower/waypoint.hpp>

#include <QWidget>

namespace Ui
{
    class WaypointWidget;
}

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            namespace gui
            {
                class WaypointWidget : public QWidget
                {
                    Q_OBJECT
                public:
                    explicit WaypointWidget(std::shared_ptr<Waypoint> waypoint, QWidget* parent=nullptr);
                    ~WaypointWidget();

                    int index() const;

                    units::Distance x() const;
                    units::Distance y() const;
                    units::Distance radius() const;
                    units::Velocity speed() const;
                    std::shared_ptr<Waypoint> waypoint() const;

                    void setIndex(int index);

                    void setX(const units::Distance& x);
                    void setY(const units::Distance& y);
                    void setRadius(const units::Distance& radius);
                    void setSpeed(const units::Velocity& speed);
                    void setWaypoint(std::shared_ptr<Waypoint> waypoint);

                private slots:
                    void updateX(const QString& text);
                    void updateY(const QString& text);
                    void updateRadius(const QString& text);
                    void updateSpeed(const QString& text);
                    void remove();

                signals:
                    void updated();

                private:
                    Ui::WaypointWidget* m_ui;
                    int m_index;
                    std::shared_ptr<Waypoint> m_waypoint;

                };
            }
        }
    }
}

#endif // WAYPOINT_WIDGET_HPP
