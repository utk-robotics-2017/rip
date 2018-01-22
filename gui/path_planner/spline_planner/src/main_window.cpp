#include <spline_planner_gui/main_window.hpp>
#include "ui_main_window.h"

#include <QDir>
#include <QStandardPaths>

namespace rip
{
    namespace gui
    {
        namespace splineplanner
        {
            MainWindow::MainWindow(QWidget* parent)
                : QMainWindow(parent)
                , m_ui(new Ui::MainWindow)
                , m_preferences_manager(Preferences::getInstance())
                , m_settings_manager(Settings::getInstance())
            {
                m_ui->setupUi(this);

                QDir app_data(QStandardPaths::writableLocation(QStandardPaths::AppDataLocation));
                if(!app_data.exists())
                {
                    app_data.mkpath(".");
                    app_data.mkpath("./robots");
                    app_data.mkpath("./courses");
                }

                connect(m_ui->course_tab, SIGNAL(polygonUpdated()), this, SLOT(courseUpdated()));

                m_preferences_manager->load();
                m_settings_manager->load();
                m_ui->robot_tab->setRobotOptions(m_settings_manager->getRobotNames());
                m_ui->course_tab->setCourseOptions(m_settings_manager->getCourseNames());
                m_ui->path_tab->setOptions(m_settings_manager->getPathNames());
            }

            MainWindow::~MainWindow()
            {
                m_preferences_manager->save();
                m_settings_manager->save();

                delete m_ui;
            }

            void MainWindow::courseUpdated()
            {
                m_ui->path_tab->setWorld(m_ui->course_tab->polygon());
            }
        }
    }
}
