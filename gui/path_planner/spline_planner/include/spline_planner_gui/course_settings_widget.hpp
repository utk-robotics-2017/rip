#ifndef COURSE_SETTINGS_WIDGET_HPP
#define COURSE_SETTINGS_WIDGET_HPP

#include <memory>
#include <vector>

#include <QString>
#include <QWidget>

#include <geometry/polygon.hpp>
#include <misc/settings_base.hpp>

#include "settings.hpp"
#include "preferences.hpp"
#include "compute_thread.hpp"

namespace Ui
{
    class CourseSettingsWidget;
}

namespace rip
{
    namespace gui
    {
        namespace splineplanner
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

                void removeCourse();

                void updateNumPoints();

                void updatePolygon();

            private:
                Ui::CourseSettingsWidget* m_ui;

                geometry::Polygon m_polygon;

                std::shared_ptr<misc::SettingsBase> m_current;
                std::shared_ptr<Settings> m_settings;
                std::shared_ptr<Preferences> m_preferences;
                std::shared_ptr<ComputeThread> m_compute_thread;
            };
        }
    }
}


#endif // COURSE_SETTINGS_WIDGET_HPP
