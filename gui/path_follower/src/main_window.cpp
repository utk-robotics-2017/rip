#include "path_follower_gui/main_window.hpp"
#include "ui_main_window.h"

#include <QFileDialog>

#include "path_follower_gui/storage.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            namespace gui
            {
                MainWindow::MainWindow(QWidget* parent)
                    : QMainWindow(parent)
                    , m_ui(new Ui::MainWindow)
                {
                    m_ui->setupUi(this);


                    connect(m_ui->import_robot, SIGNAL(triggered(bool)), this, SLOT(importRobot()));
                    connect(m_ui->import_waypoints, SIGNAL(triggered(bool)), this, SLOT(importWaypoints()));
                    connect(m_ui->import_world, SIGNAL(triggered(bool)), this, SLOT(importWorld()));
                    connect(m_ui->export_robot, SIGNAL(triggered(bool)), this, SLOT(exportRobot()));
                    connect(m_ui->export_waypoints, SIGNAL(triggered(bool)), this, SLOT(exportWaypoints()));
                    connect(m_ui->export_world, SIGNAL(triggered(bool)), this, SLOT(exportWorld()));
                }

                MainWindow::~MainWindow()
                {
                    delete m_ui;
                }

                void MainWindow::importWorld()
                {
                    QString filepath = QFileDialog::getOpenFileName(this, "Load World");
                    if (!filepath.isEmpty())
                    {
                        Storage::getInstance()->loadWorld(filepath.toStdString());
                    }
                }

                void MainWindow::exportWorld()
                {
                    /*
                    QString filepath = QFileDialog::getSaveFileName(this, "Save World");
                    if (!filepath.isEmpty())
                    {
                        Storage::getInstance()->saveWorld(filepath.toStdString());
                    }
                    */
                }

                void MainWindow::importWaypoints()
                {
                    QString filepath = QFileDialog::getOpenFileName(this, "Load Waypoints");
                    if (!filepath.isEmpty())
                    {
                        Storage::getInstance()->loadWaypoints(filepath.toStdString());
                    }
                }

                void MainWindow::exportWaypoints()
                {
                    /*
                    QString filepath = QFileDialog::getSaveFileName(this, "Save Waypoints");
                    if (!filepath.isEmpty())
                    {
                        Storage::getInstance()->saveWaypoints(filepath.toStdString());
                    }
                    */
                }

                void MainWindow::importRobot()
                {
                    QString filepath = QFileDialog::getOpenFileName(this, "Load Robot");
                    if (!filepath.isEmpty())
                    {
                        Storage::getInstance()->loadRobot(filepath.toStdString());
                    }
                }

                void MainWindow::exportRobot()
                {
                    /*
                    QString filepath = QFileDialog::getSaveFileName(this, "Save Robot");
                    if (!filepath.isEmpty())
                    {
                        Storage::getInstance()->saveRobot(filepath.toStdString());
                    }
                    */
                }
            }
        }
    }
}
