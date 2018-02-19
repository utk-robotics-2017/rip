#include <spline_planner_gui/waypoint_list_widget_item.hpp>
#include "ui_waypoint_list_widget_item.h"

#include <spline_planner_gui/preferences.hpp>

namespace rip
{
    namespace gui
    {
        namespace splineplanner
        {

            WaypointListWidgetItem::WaypointListWidgetItem(QWidget* parent)
                : QWidget(parent)
                , m_ui(new Ui::WaypointListWidgetItem)
            {
                m_ui->setupUi(this);

                connect(m_ui->x, SIGNAL(textChanged(QString)), this, SLOT(updatedText()));
                connect(m_ui->y, SIGNAL(textChanged(QString)), this, SLOT(updatedText()));
                connect(m_ui->angle, SIGNAL(textChanged(QString)), this, SLOT(updatedText()));
            }

            WaypointListWidgetItem::~WaypointListWidgetItem()
            {
                delete m_ui;
            }

            double WaypointListWidgetItem::x() const
            {
                bool ok = false;
                double d = m_ui->x->text().toDouble(&ok);
                if (ok)
                {
                    return d;
                }
                return 0.0;
            }

            void WaypointListWidgetItem::setX(double x)
            {
                m_ui->x->setText(QString::number(x));
            }

            double WaypointListWidgetItem::y() const
            {
                bool ok = false;
                double d = m_ui->y->text().toDouble(&ok);
                if (ok)
                {
                    return d;
                }
                return 0.0;
            }

            void WaypointListWidgetItem::setY(double y)
            {
                m_ui->y->setText(QString::number(y));
            }

            double WaypointListWidgetItem::angle() const
            {
                bool ok = false;
                double d = m_ui->angle->text().toDouble(&ok);
                if (ok)
                {
                    return d;
                }
                return 0.0;
            }

            void WaypointListWidgetItem::setAngle(double angle)
            {
                m_ui->angle->setText(QString::number(angle));
            }

            void WaypointListWidgetItem::setWaypoint(const navigation::Waypoint& waypoint)
            {
                std::shared_ptr<Preferences> preferences = Preferences::getInstance();
                units::Distance unit = preferences->getDistanceUnit();
                setX(waypoint.position().x().to(unit));
                setY(waypoint.position().y().to(unit));
                units::Angle angle_unit = preferences->getAngleUnit();
                setAngle(atan(waypoint.tangent()).to(angle_unit));
            }

            navigation::Waypoint WaypointListWidgetItem::waypoint() const
            {
                return navigation::Waypoint(x(), y(), angle());
            }

            bool WaypointListWidgetItem::valid() const
            {
                bool ok = false;
                m_ui->x->text().toDouble(&ok);
                if (ok)
                {
                    m_ui->y->text().toDouble(&ok);
                    if (ok)
                    {
                        m_ui->angle->text().toDouble(&ok);
                        return ok;
                    }
                }
            }

            QSize WaypointListWidgetItem::minimumSizeHint() const
            {
                return QSize(0, QWidget::minimumSizeHint().height());
            }

            QSize WaypointListWidgetItem::sizeHint() const
            {
                return QSize(0, QWidget::sizeHint().height());
            }

            void WaypointListWidgetItem::updatedText()
            {
                // Only update if text is valid number
                if(valid())
                {
                    emit modified();
                }
            }

        } // pathplanner
    } // gui
} // rip
