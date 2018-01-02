#ifndef PATH_WIDGET_HPP
#define PATH_WIDGET_HPP

#include <QWidget>

#include <polygon.hpp>

#include "preferences_manager.hpp"
#include "settings_manager.hpp"
#include "compute_thread.hpp"

namespace Ui
{
    class PathWidget;
}

namespace rip
{
    namespace gui
    {
        namespace pathplanner
        {
            class PathWidget : public QWidget
            {
                Q_OBJECT
            public:
                explicit PathWidget(QWidget* parent = nullptr);
                ~PathWidget();

                void setWorld(const geometry::Polygon& polygon);

                void setOptions(const std::vector<std::string>& options);

            private slots:
                void updatePath(const QString& name);
                void addPath();
                void updateWaypoints();
                void updateNumWaypoints();
                void updateAnimate(bool animate);
                void updateSpeedUp(QString speed_up);
                void updateDisplay(QString display);
                void updateStats();
                void updateUnits();

            private:
                Ui::PathWidget* m_ui;

                std::shared_ptr<misc::SettingsBase> m_current;
                std::shared_ptr<Settings> m_settings;
                std::shared_ptr<Preferences> m_preferences;
                std::shared_ptr<ComputeThread> m_compute_thread;
                Time m_start;
            };
        }
    }
}

#endif // PATH_WIDGET_HPP
