#ifndef ROBOT_SETTINGS_WIDGET_HPP
#define ROBOT_SETTINGS_WIDGET_HPP

#include <memory>
#include <map>
#include <string>

#include <QLineEdit>
#include <QCheckBox>

#include <misc/settings_base.hpp>

#include "settings.hpp"

namespace Ui
{
    class RobotSettingsWidget;
}

namespace rip
{
    namespace gui
    {
        namespace tebplanner
        {
            class RobotSettingsWidget : public QWidget
            {
                Q_OBJECT
            public:
                explicit RobotSettingsWidget(QWidget* parent = nullptr);

            private slots:
                void updateDistance();
                void updateVelocity();
                void updateAcceleration();
                void updateDouble();
                void updateInteger();
                void updateBool();

            private:
                std::shared_ptr<Ui::RobotSettingsWidget> m_ui;
                std::map< QLineEdit*, std::string> m_distances;
                std::map< QLineEdit*, std::string> m_velocities;
                std::map< QLineEdit*, std::string> m_accelerations;
                std::map< QLineEdit*, std::string> m_integers;
                std::map< QLineEdit*, std::string> m_doubles;
                std::map< QCheckBox*, std::string> m_bools;

                std::shared_ptr<Settings> m_settings;
                std::shared_ptr<misc::SettingsBase> m_current;
            };
        }
    }
}

#endif // ROBOT_SETTINGS_WIDGET_HPP
