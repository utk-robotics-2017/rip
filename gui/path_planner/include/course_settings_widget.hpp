#ifndef COURSE_SETTINGS_WIDGET_HPP
#define COURSE_SETTINGS_WIDGET_HPP

#include <memory>
#include <vector>

#include <QString>
#include <QWidget>

#include <polygon.hpp>

#include "settings_base.hpp"
#include "settings_manager.hpp"
#include "preferences_manager.hpp"
#include "compute_thread.hpp"

namespace Ui
{
    class CourseSettingsWidget;
}

namespace rip
{
    namespace gui
    {
        namespace pathplanner
        {
            class CourseSettingsWidget : public QWidget
            {
                Q_OBJECT
            public:
                explicit CourseSettingsWidget(QWidget* parent = nullptr);

                void setCourseOptions(const std::vector<std::string>& options);

                std::string course() const;

                geometry::Polygon polygon();

            signals:
                void polygonUpdated();

            public slots:
                void updateCourse(QString name);

                void addCourse();

                void updateNumPoints();

                void updatePolygon();

            private:
                Ui::CourseSettingsWidget* m_ui;

                geometry::Polygon m_polygon;

                std::shared_ptr<misc::SettingsBase> m_setting;
                std::shared_ptr<Settings> m_settings_manager;
                std::shared_ptr<Preferences> m_preferences_manager;
                std::shared_ptr<ComputeThread> m_compute_thread;
            };
        }
    }
}


#endif // COURSE_SETTINGS_WIDGET_HPP
