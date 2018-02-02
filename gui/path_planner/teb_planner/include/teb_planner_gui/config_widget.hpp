#ifndef CONFIG_WIDGET_HPP
#define CONFIG_WIDGET_HPP

#include <memory>
#include <map>
#include <string>

#include <QLineEdit>
#include <QCheckBox>
#include <QString>

#include <misc/settings_base.hpp>

#include "settings.hpp"

namespace Ui
{
    class ConfigWidget;
}

namespace rip
{
    namespace gui
    {
        namespace tebplanner
        {
            class ConfigWidget : public QWidget
            {
                Q_OBJECT
            public:
                explicit ConfigWidget(QWidget* parent = nullptr);

                void setOptions();
            private slots:
                void updateDistance();
                void updateTime();
                void updateVelocity();
                void updateAcceleration();
                void updateAngle();
                void updateAngularVelocity();
                void updateAngularAcceleration();
                void updateDouble();
                void updateInteger();
                void updateBool();

                void addConfig();
                void setConfig(const QString& name);
                void removeConfig();

            private:
                std::shared_ptr<Ui::ConfigWidget> m_ui;
                std::map< QLineEdit*, std::string> m_distances;
                std::map< QLineEdit*, std::string> m_times;
                std::map< QLineEdit*, std::string> m_velocities;
                std::map< QLineEdit*, std::string> m_accelerations;
                std::map< QLineEdit*, std::string> m_angular_velocities;
                std::map< QLineEdit*, std::string> m_angular_accelerations;
                std::map< QLineEdit*, std::string> m_angles;
                std::map< QLineEdit*, std::string> m_integers;
                std::map< QLineEdit*, std::string> m_doubles;
                std::map< QCheckBox*, std::string> m_bools;

                std::shared_ptr<Settings> m_settings;
                std::string m_name;
                std::shared_ptr<navigation::tebplanner::TebConfig> m_current;
            };
        }
    }
}

#endif // CONFIG_WIDGET_HPP
