#ifndef ROBOT_SETTINGS_WIDGET_HPP
#define ROBOT_SETTINGS_WIDGET_HPP

#include <vector>
#include <memory>

#include <QWidget>
#include <QString>

#include "preferences_manager.hpp"
#include "settings_manager.hpp"
#include "compute_thread.hpp"

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

                void addRobot();

                void removeRobot();

                void updateWidth(QString value);

                void updateLength(QString value);

                void updateMaxVelocity(QString value);

                void updateMaxAcceleration(QString value);

                void updateMaxJerk(QString value);

                void updateUnit();

            private:
                Ui::RobotSettingsWidget* m_ui;

                std::shared_ptr<misc::SettingsBase> m_current;
                std::shared_ptr<Preferences> m_preferences;
                std::shared_ptr<Settings> m_settings;
                std::shared_ptr<ComputeThread> m_compute_thread;
            };
        }
    }
}

#endif // ROBOT_SETTINGS_WIDGET_HPP
