#ifndef PATH_PLANNER_WIDGET_HPP
#define PATH_PLANNER_WIDGET_HPP

#include <QWidget>
#include <QFileDialog>

#include <string>
#include <memory>

namespace Ui
{
    class PlannerWidget;
}

namespace rip
{
    namespace gui
    {
        namespace tebplanner
        {
            class PlannerWidget : public QWidget
            {
                Q_OBJECT
            public:
                explicit PlannerWidget(QWidget* parent = nullptr);

            private slots:
                void setRobot();

                void setObstacles();

                void saveObstacles();

                void loadObstacles();

                void setTrajectory();

                void loadTrajectory();

                void saveTrajectory();

            private:
                std::shared_ptr<Ui::PlannerWidget> m_ui;
            };
        }
    }
}

#endif // PATH_PLANNER_WIDGET_HPP
