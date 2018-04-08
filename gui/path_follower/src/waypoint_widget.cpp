#include "path_follower_gui/waypoint_widget.hpp"
#include "ui_waypoint_widget.h"

#include <fmt/format.h>

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            namespace gui
            {

                WaypointWidget::WaypointWidget(const Waypoint& waypoint, QWidget* parent)
                    : QWidget(parent)
                    , m_ui(new Ui::WaypointWidget)
                {
                    m_ui->setupUi(this);
                    setWaypoint(waypoint);
                }

                WaypointWidget::~WaypointWidget()
                {
                    delete m_ui;
                }

                int WaypointWidget::index() const
                {
                    return m_index;
                }

                units::Distance WaypointWidget::x() const
                {
                    return m_x;
                }

                units::Distance WaypointWidget::y() const
                {
                    return m_y;
                }

                units::Distance WaypointWidget::radius() const
                {
                    return m_radius;
                }

                units::Velocity WaypointWidget::speed() const
                {
                    return m_speed;
                }

                Waypoint WaypointWidget::waypoint() const
                {
                    return Waypoint(m_x, m_y, m_radius, m_speed);
                }

                void WaypointWidget::setIndex(int index)
                {
                    m_index = index;
                    m_ui->index->setText(QString::fromStdString(fmt::format("{0:d}", index + 1)));
                }

                void WaypointWidget::setX(const units::Distance& x)
                {
                    m_x = x;
                    m_ui->x->setText(QString::fromStdString(fmt::format("{0:0.3f}", m_x.to(units::in))));
                }

                void WaypointWidget::setY(const units::Distance& y)
                {
                    m_y = y;
                    m_ui->y->setText(QString::fromStdString(fmt::format("{0:0.3f}", m_y.to(units::in))));
                }

                void WaypointWidget::setRadius(const units::Distance& radius)
                {
                    m_radius = radius;
                    m_ui->radius->setText(QString::fromStdString(fmt::format("{0:0.3f}", m_radius.to(units::in))));
                }

                void WaypointWidget::setSpeed(const units::Velocity& speed)
                {
                    m_speed = speed;
                    m_ui->speed->setText(QString::fromStdString(fmt::format("{0:0.3f}", m_speed.to(units::in/units::s))));
                }

                void WaypointWidget::setWaypoint(const Waypoint& waypoint)
                {
                    setX(waypoint.x());
                    setY(waypoint.y());
                    setRadius(waypoint.radius());
                    setSpeed(waypoint.speed());
                }

                void WaypointWidget::updateX(const QString& text)
                {
                    bool ok;
                    double value = text.toDouble(&ok);
                    if(ok)
                    {
                        m_x = value * units::in;
                        emit updated();
                    }
                }

                void WaypointWidget::updateY(const QString& text)
                {
                    bool ok;
                    double value = text.toDouble(&ok);
                    if(ok)
                    {
                        m_y = value * units::in;
                        emit updated();
                    }
                }

                void WaypointWidget::updateRadius(const QString& text)
                {
                    bool ok;
                    double value = text.toDouble(&ok);
                    if(ok)
                    {
                        m_radius = value * units::in;
                        emit updated();
                    }
                }

                void WaypointWidget::updateSpeed(const QString& text)
                {
                    bool ok;
                    double value = text.toDouble(&ok);
                    if(ok)
                    {
                        m_speed = value * units::in / units::s;
                        emit updated();
                    }
                }



            }
        }
    }
}
