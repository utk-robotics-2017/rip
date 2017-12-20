#ifndef COURSE_SETTINGS_WIDGET_HPP
#define COURSE_SETTINGS_WIDGET_HPP

#include <memory>
#include <vector>

#include <QString>
#include <QWidget>

#include "settings_base.hpp"
#include "settings_manager.hpp"

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

            public slots:
                void updateCourse(QString name);

            private:
                Ui::CourseSettingsWidget* m_ui;

                std::shared_ptr<SettingsBase> m_setting;
                std::shared_ptr<SettingsManager> m_settings_manager;
            };
        }
    }
}


#endif // COURSE_SETTINGS_WIDGET_HPP
