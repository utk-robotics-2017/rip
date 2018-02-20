#include <spline_planner_gui/robot_settings_widget.hpp>
#include "ui_robot_settings_widget.h"

#include <QInputDialog>

#include <units/units.hpp>

namespace rip
{
    namespace gui
    {
        namespace splineplanner
        {
            RobotSettingsWidget::RobotSettingsWidget(QWidget* parent)
                : QWidget(parent)
                , m_ui(new Ui::RobotSettingsWidget)
                , m_preferences(Preferences::getInstance())
                , m_settings(Settings::getInstance())
                , m_compute_thread(ComputeThread::getInstance())
            {
                m_ui->setupUi(this);

                connect(m_ui->robot_width, SIGNAL(textChanged(QString)), this, SLOT(updateWidth(QString)));
                connect(m_ui->robot_length, SIGNAL(textChanged(QString)), this, SLOT(updateLength(QString)));
                connect(m_ui->max_v, SIGNAL(textChanged(QString)), this, SLOT(updateMaxVelocity(QString)));
                connect(m_ui->max_a, SIGNAL(textChanged(QString)), this, SLOT(updateMaxAcceleration(QString)));
                connect(m_ui->max_j, SIGNAL(textChanged(QString)), this, SLOT(updateMaxJerk(QString)));
                connect(m_ui->options, SIGNAL(currentIndexChanged(QString)), this, SLOT(updateRobot(QString)));

                connect(m_ui->add, SIGNAL(clicked(bool)), this, SLOT(addRobot()));
                connect(m_ui->remove, SIGNAL(clicked(bool)), this, SLOT(removeRobot()));

                connect(m_preferences.get(), SIGNAL(distanceUnitChanged(const Distance&, const Distance&)), this, SLOT(updateUnit()));
                connect(m_preferences.get(), SIGNAL(timeUnitChanged(const Time&, const Time&)), this, SLOT(updateUnit()));

                QString distance_unit_text = QString::fromStdString(m_preferences->getDistanceUnitText());
                QString time_unit_text = QString::fromStdString(m_preferences->getTimeUnitText());

                m_ui->distance_unit->setText(distance_unit_text);
                m_ui->distance_unit_2->setText(distance_unit_text);
                m_ui->velocity_unit->setText(distance_unit_text + "/" + time_unit_text);
                m_ui->acceleration_unit->setText(distance_unit_text + "/" + time_unit_text + "^2");
                m_ui->jerk_unit->setText(distance_unit_text + "/" + time_unit_text + "^3");

                m_ui->robot_width->setEnabled(false);
                m_ui->robot_length->setEnabled(false);
                m_ui->max_v->setEnabled(false);
                m_ui->max_a->setEnabled(false);
                m_ui->max_j->setEnabled(false);
            }

            void RobotSettingsWidget::setRobotOptions(const std::vector<std::string>& options)
            {
                if(options.size())
                {
                    QStringList list;
                    for (const std::string& item : options)
                    {
                        list << QString::fromStdString(item);
                    }
                    m_ui->options->addItems(list);
                    updateRobot(QString::fromStdString(options.front()));
                }
            }

            std::string RobotSettingsWidget::robot() const
            {
                if(m_current)
                {
                    return m_current->name();
                }
                return "";
            }

            void RobotSettingsWidget::updateRobot(QString name)
            {
                m_current = m_settings->robot(name.toStdString());
                if (m_current)
                {
                    m_ui->robot_width->setEnabled(true);
                    m_ui->robot_length->setEnabled(true);
                    m_ui->max_v->setEnabled(true);
                    m_ui->max_a->setEnabled(true);
                    m_ui->max_j->setEnabled(true);

                    units::Distance d_unit = m_preferences->getDistanceUnit();
                    units::Time t_unit = m_preferences->getTimeUnit();

                    m_ui->robot_width->setText(QString::number(m_current->get<units::Distance>("width").to(d_unit)));
                    m_ui->robot_length->setText(QString::number(m_current->get<units::Distance>("length").to(d_unit)));
                    m_ui->max_v->setText(QString::number(m_current->get<units::Velocity>("max_velocity").to(d_unit / t_unit)));
                    m_ui->max_a->setText(QString::number(m_current->get<units::Acceleration>("max_acceleration").to(d_unit / t_unit / t_unit)));
                    m_ui->max_j->setText(QString::number(m_current->get<units::Jerk>("max_jerk").to(d_unit / t_unit / t_unit / t_unit)));
                    m_compute_thread->updateRobot(m_current);
                }
                else
                {
                    m_ui->robot_width->setEnabled(false);
                    m_ui->robot_length->setEnabled(false);
                    m_ui->max_v->setEnabled(false);
                    m_ui->max_a->setEnabled(false);
                    m_ui->max_j->setEnabled(false);
                }
            }

            void RobotSettingsWidget::addRobot()
            {
                bool ok;
                QString name = QInputDialog::getText(this, tr("Add Robot"), tr("Name:"), QLineEdit::Normal, "Default", &ok);
                if(ok && !name.isEmpty())
                {
                    m_settings->addRobot(name.toStdString());
                    m_ui->options->addItem(name);
                }
            }

            void RobotSettingsWidget::removeRobot()
            {
                int index = m_ui->options->currentIndex();
                if(index >= 0)
                {
                    m_settings->removeRobot(m_current->name());
                    m_ui->options->removeItem(index);
                }
            }

            void RobotSettingsWidget::updateWidth(QString value)
            {
                bool ok = false;
                double d = value.toDouble(&ok);
                if (ok)
                {
                    units::Distance width = d * m_preferences->getDistanceUnit();
                    m_current->set<units::Distance>("width", width);
                    m_compute_thread->setWidth(width);
                }
                else
                {
                    // error
                }
            }

            void RobotSettingsWidget::updateLength(QString value)
            {
                bool ok = false;
                double d = value.toDouble(&ok);
                if (ok)
                {
                    units::Distance length = d * m_preferences->getDistanceUnit();
                    m_current->set<units::Distance>("length", length);
                    m_compute_thread->setLength(length);
                }
                else
                {
                    // error
                }
            }

            void RobotSettingsWidget::updateMaxVelocity(QString value)
            {
                bool ok = false;
                double d = value.toDouble(&ok);
                if (ok)
                {
                    units::Velocity max_velocity = d * m_preferences->getDistanceUnit() / m_preferences->getTimeUnit();
                    m_current->set<units::Velocity>("max_velocity", max_velocity);
                    m_compute_thread->setMaxVelocity(max_velocity);
                }
                else
                {
                    // error
                }
            }

            void RobotSettingsWidget::updateMaxAcceleration(QString value)
            {
                bool ok = false;
                double d = value.toDouble(&ok);
                if (ok)
                {
                    units::Time t_unit = m_preferences->getTimeUnit();
                    units::Acceleration max_acceleration = d * m_preferences->getDistanceUnit() / t_unit / t_unit;
                    m_current->set<units::Acceleration>("max_acceleration", max_acceleration);
                    m_compute_thread->setMaxAcceleration(max_acceleration);
                }
                else
                {
                    // error
                }
            }

            void RobotSettingsWidget::updateMaxJerk(QString value)
            {
                bool ok = false;
                double d = value.toDouble(&ok);
                if (ok)
                {
                    units::Time t_unit = m_preferences->getTimeUnit();
                    units::Jerk max_jerk = d * m_preferences->getDistanceUnit() / t_unit / t_unit / t_unit;
                    m_current->set<units::Jerk>("max_jerk", max_jerk);
                    m_compute_thread->setMaxJerk(max_jerk);
                }
                else
                {
                    // error
                }
            }

            void RobotSettingsWidget::updateUnit()
            {
                QString distance_unit_text = QString::fromStdString(m_preferences->getDistanceUnitText());
                QString time_unit_text = QString::fromStdString(m_preferences->getTimeUnitText());

                units::Distance d_unit = m_preferences->getDistanceUnit();
                units::Time t_unit = m_preferences->getTimeUnit();

                if(m_current)
                {
                    m_ui->robot_width->setText(QString::number(m_current->get<units::Distance>("width").to(d_unit)));
                    m_ui->max_v->setText(QString::number(m_current->get<units::Velocity>("max_velocity").to(d_unit / t_unit)));
                    m_ui->max_a->setText(QString::number(m_current->get<units::Acceleration>("max_acceleration").to(d_unit / t_unit / t_unit)));
                    m_ui->max_j->setText(QString::number(m_current->get<units::Jerk>("max_jerk").to(d_unit / t_unit / t_unit / t_unit)));
                }
                m_ui->distance_unit->setText(distance_unit_text);
                m_ui->velocity_unit->setText(distance_unit_text + "/" + time_unit_text);
                m_ui->acceleration_unit->setText(distance_unit_text + "/" + time_unit_text + "^2");
                m_ui->jerk_unit->setText(distance_unit_text + "/" + time_unit_text + "^3");
            }

        }
    }
}
