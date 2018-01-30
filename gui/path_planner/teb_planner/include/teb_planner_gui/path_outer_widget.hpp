#ifndef PATH_OUTER_WIDGET_HPP
#define PATH_OUTER_WIDGET_HPP

#include <QWidget>
#include <QFileDialog>

#include <string>
#include <memory>

#include "settings.hpp"
#include "compute_thread.hpp"

namespace Ui
{
    class PathOuterWidget;
}

namespace rip
{
    namespace gui
    {
        namespace tebplanner
        {
            class PathOuterWidget : public QWidget
            {
                Q_OBJECT
            public:
                explicit PathOuterWidget(QWidget* parent = nullptr);

                void setOptions();

            private slots:
                void setRobot(const QString& text);

                void addObstacles();
                void setObstacles(const QString& text);
                void removeObstacles();

                void setConfig(const QString& text);

                void run();

            private:
                std::shared_ptr<Ui::PathOuterWidget> m_ui;
                std::shared_ptr<Settings> m_settings;
                std::shared_ptr<ComputeThread> m_compute_thread;

                std::string m_obstacles_name;

            };
        }
    }
}

#endif // PATH_OUTER_WIDGET_HPP
