#ifndef ROBOT_SETTINGS_WIDGET_HPP
#define ROBOT_SETTINGS_WIDGET_HPP

#include <vector>
#include <memory>

#include <QWidget>
#include <QString>

#include "preferences_manager.hpp"
#include "settings_manager.hpp"

namespace Ui
{
    class RobotSettingsWidget;
}

namespace rip
{
    namespace gui
    {
        namespace pathplanner
        {
            class RobotSettingsWidget : public QWidget
            {
                Q_OBJECT
            public:
                explicit RobotSettingsWidget(QWidget* parent = nullptr);

                void setRobotOptions(const std::vector<std::string>& options);

                std::string robot() const;

            public slots:
                void updateRobot(QString name);

                void updateWidth(QString value);

                void updateMaxVelocity(QString value);

                void updateMaxAcceleration(QString value);

                void updateMaxJerk(QString value);

                void updateUnit();

            private:
                RobotSettingsWidget* m_ui;

                std::shared_ptr<SettingsBase> m_settings;
                std::shared_ptr<PreferencesManager> m_preferences_manager;
                std::shared_ptr<SettingsManager> m_settings_manager;
            };
        }
    }
}

#endif // ROBOT_SETTINGS_WIDGET_HPP
