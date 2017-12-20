#include "main_window.hpp"
#include "ui_main_window.h"

#include <QDir>
#include <QStandardPaths>

namespace rip
{
    namespace gui
    {
        namespace pathplanner
        {
            MainWindow::MainWindow(QWidget* parent)
                : QMainWindow(parent)
                , m_ui(new Ui::MainWindow)
                , m_preferences_manager(PreferencesManager::getInstance())
                , m_settings_manager(SettingsManager::getInstance())
            {
                m_ui->setupUi(this);

                QDir app_data(QStandardPaths::writableLocation(QStandardPaths::AppDataLocation));
                if(!app_data.exists())
                {
                    app_data.mkpath(".");
                    app_data.mkpath("./robot");
                    app_data.mkpath("./course");
                }

                m_preferences_manager->load();
                m_settings_manager->load();
            }

            MainWindow::~MainWindow()
            {
                m_preferences_manager->save();
                m_settings_manager->save();
                nlohmann::json project;
                project["course"] = m_ui->course_tab->course();
                project["robot"] = m_ui->robot_tab->robot();

                delete m_ui;
            }
        }
    }
}
