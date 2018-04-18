#include "path_follower_gui/waypoint_list_widget_outer.hpp"
#include "ui_waypoint_list_widget_outer.h"
#include "path_follower_gui/storage.hpp"

#include <QInputDialog>

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            namespace gui
            {

                WaypointListWidgetOuter::WaypointListWidgetOuter(QWidget* parent)
                    : QWidget(parent)
                    , m_ui(new Ui::WaypointListWidgetOuter)
                {
                    m_ui->setupUi(this);


                    QStringList names;
                    for (const std::string& name : Storage::getInstance()->waypointNames())
                    {
                        names << QString::fromStdString(name);
                    }
                    m_ui->options->addItems(names);
                    if(m_ui->options->currentIndex() > -1)
                    {
                        Storage::getInstance()->selectWaypoints(m_ui->options->currentText());
                    }

                    connect(m_ui->add, SIGNAL(clicked(bool)), this, SLOT(add()));
                    connect(m_ui->remove, SIGNAL(clicked(bool)), this, SLOT(remove()));
                    connect(m_ui->add_waypoint, SIGNAL(clicked(bool)), this, SLOT(addWaypoint()));
                    connect(m_ui->options, SIGNAL(currentTextChanged(QString)), Storage::getInstance().get(), SLOT(selectWaypoints(QString)));
                    connect(Storage::getInstance().get(), SIGNAL(selectedWaypointsChanged()), this, SLOT(updateWaypoints()));
                    connect(Storage::getInstance().get(), SIGNAL(waypointsOptionsChanged()), this, SLOT(waypointsOptionsChanged()));
                }

                WaypointListWidgetOuter::~WaypointListWidgetOuter()
                {
                    delete m_ui;
                }

                void WaypointListWidgetOuter::waypointsOptionsChanged()
                {
                    disconnect(m_ui->options, SIGNAL(currentTextChanged(QString)), this, SLOT(updateWaypoints()));

                    // todo: handle add and remove better

                    QString current_text = m_ui->options->currentText();

                    QStringList names;
                    for (const std::string& name : Storage::getInstance()->waypointNames())
                    {
                        names << QString::fromStdString(name);
                    }

                    m_ui->options->clear();
                    m_ui->options->addItems(names);
                    m_ui->options->setCurrentText(current_text);

                    connect(m_ui->options, SIGNAL(currentTextChanged(QString)), this, SLOT(updateWaypoints()));
                }

                void WaypointListWidgetOuter::add()
                {
                    bool ok;
                    QString name = QInputDialog::getText(this, tr("Add Waypoints"), tr("Name:"), QLineEdit::Normal, "Default", &ok);
                    if (ok && !name.isEmpty())
                    {
                        Storage::getInstance()->addWaypoints(name.toStdString());
                    }
                }

                void WaypointListWidgetOuter::remove()
                {
                    Storage::getInstance()->removeWaypoints(m_ui->options->currentText().toStdString());
                }

                void WaypointListWidgetOuter::addWaypoint()
                {
                    if(m_waypoints)
                    {
                        m_waypoints->addWaypoint(Waypoint(0,0,0,0));
                        m_ui->inner->updateWaypoints();
                    }
                }

                void WaypointListWidgetOuter::updateWaypoints()
                {
                    m_waypoints = Storage::getInstance()->selectedWaypoints();
                }
            }
        }
    }
}
