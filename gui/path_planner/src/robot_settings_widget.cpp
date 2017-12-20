#include "robot_settings_widget.hpp"
#include "ui_robot_settings_widget.h"

#include <units.hpp>

namespace rip
{
    namespace gui
    {
        namespace pathplanner
        {
            RobotSettingsWidget::RobotSettingsWidget(QWidget* parent)
                : QWidget(parent)
                , m_preferences_manager(PreferencesManager::getInstance())
                , m_settings_manager(SettingsManager::getInstance())
            {
                connect(m_ui->width, SIGNAL(textChanged(QString)), this, SLOT(updateWidth(QString)));
                connect(m_ui->max_v, SIGNAL(textChanged(QString)), this, SLOT(updateMaxVelocity(QString)));
                connect(m_ui->max_a, SIGNAL(textChanged(QString)), this, SLOT(updateMaxAcceleration(QString)));
                connect(m_ui->max_j, SIGNAL(textChanged(QString)), this, SLOT(updateMaxJerk(QString)));
                connect(m_ui->options, SIGNAL(currentIndexText(QString)), this, SLOT(updateRobot(QString)));

                connect(m_preferences_manager.get(), SIGNAL(distanceUnitChanged(const Distance&, const Distance&)), this, SLOT(updateUnit()));
                connect(m_preferences_manager.get(), SIGNAL(timeUnitChanged(const Time&, const Time&)), this, SLOT(updateUnit()));
            }

            void RobotSettingsWidget::setRobotOptions(const std::vector<std::string>& options)
            {
                QStringList list;
                for (const std::string& item : options)
                {
                    list << QString::fromStdString(item);
                }
                m_ui->options->addItems(list);
            }

            std::string RobotSettingsWidget::robot() const
            {
                if(m_settings)
                {
                    return m_settings->name();
                }
                return "";
            }

            void RobotSettingsWidget::updateRobot(QString name)
            {
                m_settings = m_settings_manager->robot(name.toStdString());
                if (m_settings)
                {
                    m_ui->width->setEnabled(true);
                    m_ui->max_v->setEnabled(true);
                    m_ui->max_a->setEnabled(true);
                    m_ui->max_j->setEnabled(true);
                }
                else
                {
                    m_ui->width->setEnabled(false);
                    m_ui->max_v->setEnabled(false);
                    m_ui->max_a->setEnabled(false);
                    m_ui->max_j->setEnabled(false);
                }
            }

            void RobotSettingsWidget::updateWidth(QString value)
            {
                bool ok = false;
                double d = value.toDouble(&ok);
                if (ok)
                {
                    m_settings["width"] = d * m_preferences_manager->getDistanceUnit();
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
                    m_settings["max_velocity"] = d * m_preferences_manager->getDistanceUnit() / m_preferences_manager->getTimeUnit();
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
                    units::Time t_unit = m_preferences_manager->getTimeUnit();
                    m_settings["max_acceleration"] = d * m_preferences_manager->getDistanceUnit() / t_unit / t_unit;
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
                    units::Time t_unit = m_preferences_manager->getTimeUnit();
                    m_settings["max_jerk"] = d * m_preferences_manager->getDistanceUnit() / t_unit / t_unit / t_unit;
                }
                else
                {
                    // error
                }
            }

            void RobotSettingsWidget::updateUnit()
            {
                std::string distance_unit_text = m_preferences_manager->getDistanceUnitText();
                std::string time_unit_text = m_preferences_manager->getTimeUnitText();

                m_ui->distance_unit->setText(distance_unit_text);
                m_ui->velocity_unit->setText(distance_unit_text + "/" + time_unit_text);
                m_ui->acceleration_unit->setText(distance_unit_text + "/" + time_unit_text + "^2");
                m_ui->jerk_unit->setText(distance_unit_text + "/" + time_unit_text + "^3");
            }

        }
    }
}
