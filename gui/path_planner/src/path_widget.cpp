#include "path_widget.hpp"
#include "ui_path_widget.h"

#include <tuple>

#include <QInputDialog>

#include "waypoint_list_widget_item.hpp"

namespace rip
{
    namespace gui
    {
        namespace pathplanner
        {
            PathWidget::PathWidget(QWidget* parent)
                : QWidget(parent)
                , m_ui(new Ui::PathWidget)
                , m_preferences(Preferences::getInstance())
                , m_settings(Settings::getInstance())
                , m_compute_thread(ComputeThread::getInstance())
            {
                m_ui->setupUi(this);

                m_ui->speed_up->addItems(QStringList() << "4x" << "2x" << "1x" << "1/2x" << "1/4x");
                m_ui->speed_up->setCurrentText("1x");

                m_ui->display->addItems(QStringList() << "Center" << "Center & Sides" << "Robot");

                connect(m_compute_thread.get(), SIGNAL(newPlan()), this, SLOT(updateStats()));
                connect(m_preferences.get(), SIGNAL(distanceUnitChanged(Distance, Distance)), this, SLOT(updateStats()));
                connect(m_preferences.get(), SIGNAL(timeUnitChanged(Time, Time)), this, SLOT(updateStats()));

                connect(m_ui->num_waypoints, SIGNAL(textChanged(QString)), this, SLOT(updateNumWaypoints()));
                connect(m_ui->animate, SIGNAL(toggled(bool)), this, SLOT(updateAnimate(bool)));
                connect(m_ui->speed_up, SIGNAL(currentIndexChanged(QString)), this, SLOT(updateSpeedUp(QString)));
                connect(m_ui->display, SIGNAL(currentIndexChanged(QString)), this, SLOT(updateDisplay(QString)));
                connect(m_ui->options, SIGNAL(currentIndexChanged(QString)), this, SLOT(updatePath(QString)));
                connect(m_ui->add, SIGNAL(clicked(bool)), this, SLOT(addPath()));
            }

            PathWidget::~PathWidget()
            {
                delete m_ui;
            }

            void PathWidget::setWorld(const geometry::Polygon& polygon)
            {
                m_ui->figure->setWorld(polygon);
            }

            void PathWidget::setOptions(const std::vector<std::string>& options)
            {
                if(options.size())
                {
                    QStringList list;
                    for (const std::string& item : options)
                    {
                        list << QString::fromStdString(item);
                    }
                    m_ui->options->addItems(list);
                }
            }

            void PathWidget::updatePath(const QString& name)
            {
                // TODO: delete any old list widgets
                m_ui->waypoint_widget->clear();
                m_current = m_settings->path(name.toStdString());
                if(m_current)
                {
                    nlohmann::json waypoints = m_current->get<nlohmann::json>("waypoints");
                    for(auto waypoint : waypoints)
                    {
                        WaypointListWidgetItem* wlwi = new WaypointListWidgetItem(m_ui->waypoint_widget);
                        navigation::pathplanner::Waypoint w = waypoint;
                        wlwi->setWaypoint(w);
                        wlwi->resize(m_ui->waypoint_widget->size());
                        wlwi->show();
                        QListWidgetItem* lwi = new QListWidgetItem(m_ui->waypoint_widget);
                        lwi->setSizeHint(wlwi->sizeHint());
                        m_ui->waypoint_widget->addItem(lwi);
                        m_ui->waypoint_widget->setItemWidget(lwi, wlwi);
                        connect(wlwi, SIGNAL(modified()), this, SLOT(updateWaypoints()));
                    }
                    m_ui->num_waypoints->setText(QString::number(waypoints.size()));
                    updateWaypoints();
                }
            }

            void PathWidget::addPath()
            {
                bool ok;
                QString name = QInputDialog::getText(this, tr("Add Path"), tr("Name:"), QLineEdit::Normal, "Default", &ok);
                if(ok && !name.isEmpty())
                {
                    m_settings->addPath(name.toStdString());
                    m_ui->options->addItem(name);
                    m_ui->options->setCurrentText(name);
                }
            }

            void PathWidget::updateWaypoints()
            {
                std::vector<navigation::pathplanner::Waypoint> waypoints;
                units::Distance d_unit = m_preferences->getDistanceUnit();
                units::Angle a_unit = m_preferences->getAngleUnit();
                for(int i = 0, end = m_ui->waypoint_widget->count(); i < end; i++)
                {
                    QListWidgetItem* lwi = m_ui->waypoint_widget->item(i);
                    WaypointListWidgetItem* wlwi = static_cast<WaypointListWidgetItem*>(m_ui->waypoint_widget->itemWidget(lwi));

                    if(wlwi->valid())
                    {
                        navigation::pathplanner::Waypoint waypoint(wlwi->x() * d_unit, wlwi->y() * d_unit, wlwi->angle() * a_unit);
                        waypoints.push_back(waypoint);
                    }
                }
                m_current->set< std::vector< navigation::pathplanner::Waypoint > >("waypoints", waypoints);
                m_compute_thread->setWaypoints(waypoints);
            }

            void PathWidget::updateNumWaypoints()
            {
                bool ok = false;
                int num_points = m_ui->num_waypoints->text().toInt(&ok);
                if(ok)
                {
                    int count = m_ui->waypoint_widget->count();
                    if(num_points > count)
                    {
                        // Increase the number of rows
                        for(int i = count; i < num_points; i++)
                        {
                            WaypointListWidgetItem* wlwi = new WaypointListWidgetItem(m_ui->waypoint_widget);
                            wlwi->resize(m_ui->waypoint_widget->size());
                            wlwi->show();
                            QListWidgetItem* lwi = new QListWidgetItem(m_ui->waypoint_widget);
                            lwi->setSizeHint(wlwi->sizeHint());
                            m_ui->waypoint_widget->addItem(lwi);
                            m_ui->waypoint_widget->setItemWidget(lwi, wlwi);
                            connect(wlwi, SIGNAL(modified()), this, SLOT(updateWaypoints()));
                        }
                    }
                    else if(num_points < count)
                    {
                        // Decrease number of rows
                        for(int i = count - 1; i >= num_points; i--)
                        {
                            QListWidgetItem* lwi = m_ui->waypoint_widget->item(i);
                            delete m_ui->waypoint_widget->itemWidget(lwi);
                            delete m_ui->waypoint_widget->takeItem(i);
                        }
                        updateWaypoints();
                    }
                }
            }

            void PathWidget::updateAnimate(bool animate)
            {
                m_ui->figure->setAnimate(animate);
                if(animate)
                {
                    m_ui->speed_up->setEnabled(true);
                }
                else
                {
                    m_ui->speed_up->setEnabled(false);
                }
            }
            
            void PathWidget::updateSpeedUp(QString speed_up)
            {
                if(speed_up == "1x")
                {
                    m_ui->figure->setSpeedUp(1);
                }
                else if(speed_up == "2x")
                {
                    m_ui->figure->setSpeedUp(2);
                }
                else if(speed_up == "4x")
                {
                    m_ui->figure->setSpeedUp(4);
                }
                else if(speed_up == "1/2x")
                {
                    m_ui->figure->setSpeedUp(0.5);
                }
                else if(speed_up == "1/4x")
                {
                    m_ui->figure->setSpeedUp(0.25);
                }
            }

            void PathWidget::updateDisplay(QString display)
            {
                m_ui->figure->setDisplay(display);
            }

            void PathWidget::updateStats()
            {
                std::tuple<Distance, Time> stats = m_compute_thread->stats();
                m_ui->length->setText(QString::number(std::get<0>(stats).to(m_preferences->getDistanceUnit()), 'f', 2));
                m_ui->time->setText(QString::number(std::get<1>(stats).to(m_preferences->getTimeUnit()), 'f', 2));
            }

            void PathWidget::updateUnits()
            {
                m_ui->distance_unit->setText(QString::fromStdString(m_preferences->getDistanceUnitText()));
                m_ui->time_units->setText(QString::fromStdString(m_preferences->getTimeUnitText()));
            }


        }
    }
}

