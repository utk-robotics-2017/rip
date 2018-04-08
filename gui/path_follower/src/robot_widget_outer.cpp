#include "path_follower_gui/robot_widget_outer.hpp"
#include "ui_robot_widget_outer.h"
#include "path_follower_gui/storage.hpp"

#include <fmt/format.h>

#include <QInputDialog>

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            namespace gui
            {
                RobotWidgetOuter::RobotWidgetOuter(QWidget* parent)
                    : QWidget(parent)
                    , m_ui(new Ui::RobotWidgetOuter)
                {
                    m_ui->setupUi(this);

                    connect(Storage::getInstance().get(), SIGNAL(selectedRobotChanged()), this, SLOT(updateRobot()));

                    QStringList names;
                    for (const std::string& name : Storage::getInstance()->robotNames())
                    {
                        names << QString::fromStdString(name);
                    }
                    m_ui->options->addItems(names);
                    if (m_ui->options->currentIndex() > -1)
                    {
                        Storage::getInstance()->selectRobot(m_ui->options->currentText());
                    }

                    connect(m_ui->options, SIGNAL(currentTextChanged(QString)), Storage::getInstance().get(), SLOT(selectRobot(QString)));
                    connect(m_ui->add, SIGNAL(clicked(bool)), this, SLOT(add()));
                    connect(m_ui->remove, SIGNAL(clicked(bool)), this, SLOT(remove()));
                    connect(Storage::getInstance().get(), SIGNAL(robotOptionsChanged()), this, SLOT(robotOptionsChanged()));

                    connect(m_ui->max_speed, SIGNAL(textEdited(QString)), this, SLOT(maxSpeedChanged(QString)));
                    connect(m_ui->max_acceleration, SIGNAL(textEdited(QString)), this, SLOT(maxAccelerationChanged(QString)));
                    connect(m_ui->wheelbase, SIGNAL(textEdited(QString)), this, SLOT(wheelbaseChanged(QString)));
                }

                RobotWidgetOuter::~RobotWidgetOuter()
                {
                    delete m_ui;
                }

                void RobotWidgetOuter::robotOptionsChanged()
                {
                    disconnect(m_ui->options, SIGNAL(currentTextChanged(QString)), this, SLOT(robotChanged(QString)));

                    // todo: handle add and remove better

                    QString current_text = m_ui->options->currentText();

                    QStringList names;
                    for (const std::string& name : Storage::getInstance()->robotNames())
                    {
                        names << QString::fromStdString(name);
                    }

                    m_ui->options->clear();
                    m_ui->options->addItems(names);
                    m_ui->options->setCurrentText(current_text);

                    connect(m_ui->options, SIGNAL(currentTextChanged(QString)), this, SLOT(robotChanged(QString)));
                }

                void RobotWidgetOuter::updateRobot()
                {
                    m_robot = Storage::getInstance()->selectedRobot();
                    m_ui->max_speed->setText(QString::fromStdString(fmt::format("{0:0.3f}", m_robot->maxSpeed().to(units::in/units::s))));
                    m_ui->max_acceleration->setText(QString::fromStdString(fmt::format("{0:0.3f}", m_robot->maxAcceleration().to(units::in/units::s/units::s))));
                    m_ui->wheelbase->setText(QString::fromStdString(fmt::format("{0:0.3f}", m_robot->wheelbase().to(units::in))));
                }

                void RobotWidgetOuter::add()
                {
                    bool ok;
                    QString name = QInputDialog::getText(this, tr("Add Robot"), tr("Name:"), QLineEdit::Normal, "Default", &ok);
                    if (ok && !name.isEmpty())
                    {
                        m_ui->inner->setRobot(Storage::getInstance()->addRobot(name.toStdString()), true);
                    }
                }

                void RobotWidgetOuter::remove()
                {
                    Storage::getInstance()->removeRobot(m_ui->options->currentText().toStdString());
                }

                void RobotWidgetOuter::maxSpeedUpdated(const QString& text)
                {
                    if(m_robot)
                    {
                        bool ok;
                        double value = text.toDouble(&ok);
                        if(ok)
                        {
                            m_robot->setMaxSpeed(value * units::in / units::s);
                        }
                    }
                }

                void RobotWidgetOuter::maxAccelerationUpdated(const QString& text)
                {
                    if(m_robot)
                    {
                        bool ok;
                        double value = text.toDouble(&ok);
                        if(ok)
                        {
                            m_robot->setMaxAcceleration(value * units::in / units::s / units::s);
                        }
                    }
                }

                void RobotWidgetOuter::wheelbaseUpdated(const QString& text)
                {
                    if(m_robot)
                    {
                        bool ok;
                        double value = text.toDouble(&ok);
                        if(ok)
                        {
                            m_robot->setWheelbase(value * units::in);
                        }
                    }
                }
            }
        }
    }
}
