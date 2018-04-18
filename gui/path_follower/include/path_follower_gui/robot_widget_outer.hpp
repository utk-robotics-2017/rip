#ifndef ROBOT_WIDGET_OUTER_HPP
#define ROBOT_WIDGET_OUTER_HPP

#include <memory>

#include <QWidget>

#include "path_follower_gui/robot.hpp"

namespace Ui
{
    class RobotWidgetOuter;
}


namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            namespace gui
            {
                class RobotWidgetOuter : public QWidget
                {
                    Q_OBJECT
                public:
                    explicit RobotWidgetOuter(QWidget* parent = nullptr);
                    ~RobotWidgetOuter();

                public slots:
                    void robotOptionsChanged();
                    void updateRobot();

                private slots:
                    void add();
                    void remove();
                    void maxSpeedUpdated(const QString& text);
                    void maxAccelerationUpdated(const QString& text);
                    void wheelbaseUpdated(const QString& text);

                private:
                    Ui::RobotWidgetOuter* m_ui;
                    std::shared_ptr<Robot> m_robot;
                };
            }
        }
    }
}
#endif // ROBOT_WIDGET_OUTER_HPP
