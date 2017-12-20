#include "course_settings_widget.hpp"
#include "ui_course_settings_widget.h"

namespace rip
{
    namespace gui
    {
        namespace pathplanner
        {
            CourseSettingsWidget::CourseSettingsWidget(QWidget* parent)
                : QWidget(parent)
                , m_ui(new Ui::CourseSettingsWidget)
                , m_settings_manager(SettingsManager::getInstance())
            {
                m_ui->setupUi(this);
                connect(m_ui->options, SIGNAL(currentIndexText(QString)), this, SLOT(updateCourse(QString)));
            }

            void CourseSettingsWidget::setCourseOptions(const std::vector<std::string>& options)
            {
                QStringList list;
                for (const std::string& item : options)
                {
                    list << QString::fromStdString(item);
                }
                m_ui->options->addItems(list);
            }

            std::string CourseSettingsWidget::course() const
            {
                if(m_setting)
                {
                    return m_setting->name();
                }
                return "";
            }

            void CourseSettingsWidget::updateCourse(QString name)
            {
                m_setting = m_settings_manager->course(name.toStdString());
            }
        }
    }
}
