#include <teb_planner_gui/path_outer_widget.hpp>
#include "ui_path_outer_widget.h"

#include <QInputDialog>
#include <QFile>

namespace rip
{
    namespace gui
    {
        namespace tebplanner
        {

            PathOuterWidget::PathOuterWidget(QWidget* parent)
                : QWidget(parent)
                , m_ui(new Ui::PathOuterWidget)
                , m_settings(Settings::getInstance())
                , m_compute_thread(ComputeThread::getInstance())
            {
                m_ui->setupUi(this);

                connect(m_ui->obstacle_options, SIGNAL(currentIndexChanged(QString)), this, SLOT(setObstacles(QString)));
                connect(m_ui->add_obstacles, SIGNAL(clicked()), this, SLOT(addObstacles()));
                connect(m_ui->delete_obstacles, SIGNAL(clicked()), this, SLOT(removeObstacles()));

                connect(m_ui->robot_options, SIGNAL(currentIndexChanged(QString)), this, SLOT(setRobot(QString)));

                connect(m_ui->config_options, SIGNAL(currentIndexChanged(QString)), this, SLOT(setConfig(QString)));

                connect(m_ui->run, SIGNAL(clicked(bool)), this, SLOT(run()));

                connect(m_settings.get(), SIGNAL(robotAdded(QString)), this, SLOT(robotAdded(QString)));
                connect(m_settings.get(), SIGNAL(robotRemoved(QString)), this, SLOT(robotRemoved(QString)));
                connect(m_settings.get(), SIGNAL(configAdded(QString)), this, SLOT(configAdded(QString)));
                connect(m_settings.get(), SIGNAL(configRemoved(QString)), this, SLOT(configRemoved(QString)));
            }

            void PathOuterWidget::setOptions()
            {
                QStringList names;
                for(const std::string& name : m_settings->getRobotNames())
                {
                    names << name.c_str();
                }
                m_ui->robot_options->addItems(names);

                names.clear();
                for(const std::string& name : m_settings->getConfigNames())
                {
                    names << name.c_str();
                }
                m_ui->config_options->addItems(names);

                names.clear();
                for(const std::string& name : m_settings->getObstaclesNames())
                {
                    names << name.c_str();
                }
                m_ui->obstacle_options->addItems(names);
            }

            void PathOuterWidget::saveTrajectory(const std::string& filepath)
            {
                std::vector< navigation::tebplanner::TrajectoryPoint > trajectory = m_compute_thread->trajectory();
                nlohmann::json j = trajectory;
                QFile f(QString::fromStdString(filepath));
                if(f.open(QIODevice::WriteOnly))
                {
                    f.write(j.dump(4).c_str());
                }
            }

            void PathOuterWidget::setRobot(const QString& text)
            {
                std::string name = text.toStdString();
                std::shared_ptr<navigation::tebplanner::PolygonRobotFootprint> robot = m_settings->robot(name);
                m_compute_thread->setRobot(robot);
            }

            void PathOuterWidget::addObstacles()
            {
                bool ok;
                QString name = QInputDialog::getText(this, tr("Add Obstacles"), tr("Name:"), QLineEdit::Normal, "Default", &ok);
                if (ok && !name.isEmpty())
                {
                    m_obstacles_name = name.toStdString();
                    std::shared_ptr< std::vector< std::shared_ptr<navigation::tebplanner::Obstacle> > > obstacles = m_settings->addObstacles(m_obstacles_name);
                    m_compute_thread->setObstacles(obstacles);
                    m_ui->widget->setObstacles(obstacles);
                    m_ui->obstacle_options->addItem(QString::fromStdString(m_obstacles_name));
                }
            }

            void PathOuterWidget::setObstacles(const QString& text)
            {
                m_obstacles_name = text.toStdString();
                if(m_obstacles_name.empty())
                {
                    m_compute_thread->setObstacles(nullptr);
                    m_ui->widget->setObstacles(nullptr);
                }
                else
                {
                    std::shared_ptr< std::vector< std::shared_ptr<navigation::tebplanner::Obstacle> > > obstacles = m_settings->obstacles(m_obstacles_name);
                    m_compute_thread->setObstacles(obstacles);
                    m_ui->widget->setObstacles(obstacles);
                }
            }

            void PathOuterWidget::removeObstacles()
            {
                m_settings->removeObstacles(m_obstacles_name);
                m_obstacles_name = "";
                m_compute_thread->setObstacles(nullptr);
                m_ui->widget->setObstacles(nullptr);
                m_ui->obstacle_options->removeItem(m_ui->obstacle_options->currentIndex());
            }

            void PathOuterWidget::robotAdded(const QString& name)
            {
                m_ui->robot_options->addItem(name);
            }

            void PathOuterWidget::robotRemoved(const QString& name)
            {
                int index = m_ui->robot_options->findText(name);
                if(index > -1)
                {
                    m_ui->robot_options->removeItem(index);
                }
            }

            void PathOuterWidget::configAdded(const QString& name)
            {
                m_ui->config_options->addItem(name);
            }

            void PathOuterWidget::configRemoved(const QString& name)
            {
                int index = m_ui->config_options->findText(name);
                if(index > -1)
                {
                    m_ui->config_options->removeItem(index);
                }
            }

            void PathOuterWidget::setConfig(const QString& text)
            {
                std::string name = text.toStdString();
                std::shared_ptr< navigation::tebplanner::TebConfig > config = m_settings->config(name);
                m_compute_thread->setConfig(config);
            }

            void PathOuterWidget::run()
            {
                m_compute_thread->start();
            }

        }
    }
}
