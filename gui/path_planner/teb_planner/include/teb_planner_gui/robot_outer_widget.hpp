#ifndef ROBOT_OUTER_WIDGET_HPP
#define ROBOT_OUTER_WIDGET_HPP

#include <memory>

#include <QWidget>

#include "teb_planner_gui/settings.hpp"

namespace Ui
{
    class RobotOuterWidget;
}

namespace rip
{
    namespace gui
    {
        namespace tebplanner
        {
            class RobotOuterWidget : public QWidget
            {
                Q_OBJECT
            public:
                explicit RobotOuterWidget(QWidget* parent = nullptr);

                void setOptions();
            private slots:
                void addRobot();
                void setRobot(const QString& text);
                void removeRobot();

            private:
                std::shared_ptr<Ui::RobotOuterWidget> m_ui;
                std::shared_ptr<Settings> m_settings;
                std::string m_name;
            };
        }
    }
}

#endif // ROBOT_OUTER_WIDGET_HPP
