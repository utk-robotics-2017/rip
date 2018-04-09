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

                WaypointWidget::WaypointWidget(std::shared_ptr<Waypoint> waypoint, QWidget* parent)
                    : QWidget(parent)
                    , m_ui(new Ui::WaypointWidget)
                {
                    m_ui->setupUi(this);
                    setWaypoint(waypoint);

                    connect(m_ui->x, SIGNAL(textEdited(QString)), this, SLOT(updateX(QString)));
                    connect(m_ui->y, SIGNAL(textEdited(QString)), this, SLOT(updateY(QString)));
                    connect(m_ui->radius, SIGNAL(textEdited(QString)), this, SLOT(updateRadius(QString)));
                    connect(m_ui->speed, SIGNAL(textEdited(QString)), this, SLOT(updateSpeed(QString)));
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
                    return m_waypoint->x();
                }

                units::Distance WaypointWidget::y() const
                {
                    return m_waypoint->y();
                }

                units::Distance WaypointWidget::radius() const
                {
                    return m_waypoint->radius();
                }

                units::Velocity WaypointWidget::speed() const
                {
                    return m_waypoint->speed();
                }

                std::shared_ptr<Waypoint> WaypointWidget::waypoint() const
                {
                    return m_waypoint;
                }

                void WaypointWidget::setIndex(int index)
                {
                    m_index = index;
                    m_ui->index->setText(QString::fromStdString(fmt::format("{0:d}", index + 1)));
                }

                void WaypointWidget::setX(const units::Distance& x)
                {
                    m_waypoint->setX(x);
                    m_ui->x->setText(QString::fromStdString(fmt::format("{0:0.3f}", m_waypoint->x().to(units::in))));
                }

                void WaypointWidget::setY(const units::Distance& y)
                {
                    m_waypoint->setY(y);
                    m_ui->y->setText(QString::fromStdString(fmt::format("{0:0.3f}", m_waypoint->y().to(units::in))));
                }

                void WaypointWidget::setRadius(const units::Distance& radius)
                {
                    m_waypoint->setRadius(radius);
                    m_ui->radius->setText(QString::fromStdString(fmt::format("{0:0.3f}", m_waypoint->radius().to(units::in))));
                }

                void WaypointWidget::setSpeed(const units::Velocity& speed)
                {
                    m_waypoint->setSpeed(speed);
                    m_ui->speed->setText(QString::fromStdString(fmt::format("{0:0.3f}", m_waypoint->speed().to(units::in/units::s))));
                }

                void WaypointWidget::setWaypoint(std::shared_ptr<Waypoint> waypoint)
                {
                    m_waypoint = waypoint;
                    m_ui->x->setText(QString::fromStdString(fmt::format("{0:0.3f}", m_waypoint->x().to(units::in))));
                    m_ui->y->setText(QString::fromStdString(fmt::format("{0:0.3f}", m_waypoint->y().to(units::in))));
                    m_ui->radius->setText(QString::fromStdString(fmt::format("{0:0.3f}", m_waypoint->radius().to(units::in))));
                    m_ui->speed->setText(QString::fromStdString(fmt::format("{0:0.3f}", m_waypoint->speed().to(units::in/units::s))));
                }

                void WaypointWidget::updateX(const QString& text)
                {
                    bool ok;
                    double value = text.toDouble(&ok);
                    if(ok)
                    {
                        m_waypoint->setX(value * units::in);
                        emit updated();
                    }
                }

                void WaypointWidget::updateY(const QString& text)
                {
                    bool ok;
                    double value = text.toDouble(&ok);
                    if(ok)
                    {
                        m_waypoint->setY(value * units::in);
                        emit updated();
                    }
                }

                void WaypointWidget::updateRadius(const QString& text)
                {
                    bool ok;
                    double value = text.toDouble(&ok);
                    if(ok)
                    {
                        m_waypoint->setRadius(value * units::in);
                        emit updated();
                    }
                }

                void WaypointWidget::updateSpeed(const QString& text)
                {
                    bool ok;
                    double value = text.toDouble(&ok);
                    if(ok)
                    {
                        m_waypoint->setSpeed(value * units::in / units::s);
                        emit updated();
                    }
                }



            }
        }
    }
}
