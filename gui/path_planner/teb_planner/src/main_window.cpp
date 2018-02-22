#include <teb_planner_gui/main_window.hpp>
#include "ui_main_window.h"

#include <QFileDialog>

namespace rip
{
    namespace gui
    {
        namespace tebplanner
        {
            MainWindow::MainWindow(QWidget* parent)
                : QMainWindow(parent)
                , m_ui(new Ui::MainWindow)
                , m_settings(Settings::getInstance())
            {
                m_ui->setupUi(this);
                m_settings->load();

                m_ui->config->setOptions();
                m_ui->robot->setOptions();
                m_ui->planner->setOptions();

                connect(m_ui->import_robot, SIGNAL(triggered(bool)), this, SLOT(importRobot()));
                connect(m_ui->export_robot, SIGNAL(triggered(bool)), this, SLOT(exportRobot()));
                connect(m_ui->import_config, SIGNAL(triggered(bool)), this, SLOT(importConfig()));
                connect(m_ui->export_config, SIGNAL(triggered(bool)), this, SLOT(exportConfig()));
                connect(m_ui->import_obstacles, SIGNAL(triggered(bool)), this, SLOT(importObstacles()));
                connect(m_ui->export_obstacles, SIGNAL(triggered(bool)), this, SLOT(exportObstacles()));
                connect(m_ui->export_trajectory, SIGNAL(triggered(bool)), this, SLOT(exportTrajectory()));
            }

            void MainWindow::importRobot()
            {
                QString filepath = QFileDialog::getOpenFileName(this, "Load Robot");
               if(!filepath.isEmpty())
               {
                   m_settings->loadRobot(filepath.toStdString());
               }
            }

            void MainWindow::exportRobot()
            {
                QString filepath = QFileDialog::getSaveFileName(this, "Save Robot");
                if(!filepath.isEmpty())
                {
                    m_settings->saveRobot(filepath.toStdString());
                }
            }

            void MainWindow::importConfig()
            {
                QString filepath = QFileDialog::getOpenFileName(this, "Load Config");
                if(!filepath.isEmpty())
                {
                    m_settings->loadConfig(filepath.toStdString());
                }
            }

            void MainWindow::exportConfig()
            {
                QString filepath = QFileDialog::getSaveFileName(this, "Save Config");
                if(!filepath.isEmpty())
                {
                    m_settings->saveConfig(filepath.toStdString());
                }
            }

            void MainWindow::importObstacles()
            {
                QString filepath = QFileDialog::getOpenFileName(this, "Load Obstacles");
                if(!filepath.isEmpty())
                {
                    m_settings->loadObstacles(filepath.toStdString());
                }
            }

            void MainWindow::exportObstacles()
            {
                QString filepath = QFileDialog::getSaveFileName(this, "Save Obstacles");
                if(!filepath.isEmpty())
                {
                    m_settings->saveObstacles(filepath.toStdString());
                }
            }

            void MainWindow::exportTrajectory()
            {
                QString filepath = QFileDialog::getSaveFileName(this, "Save Trajectory");
                if(!filepath.isEmpty())
                {
                    m_ui->planner->saveTrajectory(filepath.toStdString());
                }
            }

            MainWindow::~MainWindow()
            {
                m_settings->save();
            }
        }
    }
}
