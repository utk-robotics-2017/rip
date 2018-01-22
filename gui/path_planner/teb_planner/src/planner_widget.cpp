#include <teb_planner_gui/planner_widget.hpp>
#include "ui_planner_widget.h"

namespace rip
{
    namespace gui
    {
        namespace tebplanner
        {

            PlannerWidget::PlannerWidget(QWidget* parent)
                : QWidget(parent)
                , m_ui(new Ui::PlannerWidget)
            {
                m_ui->setupUi(this);

                /*
                connect(m_ui->trajectory_options, SIGNAL(currentIndexChanged(QString)), this, SLOT(setTrajectory()));
                connect(m_ui->load_trajectory, SIGNAL(clicked()), this, SLOT(loadTrajectory()));
                connect(m_ui->save_trajectory, SIGNAL(clicked()), this, SLOT(saveTrajectory()));
                connect(m_ui->add_trajectory, SIGNAL(clicked()), this, SLOT(addTrajectory()));
                connect(m_ui->delete_trajectory, SIGNAL(clicked()), this, SLOT(deleteTrajectory()));

                connect(m_ui->obstacle_options, SIGNAL(currentIndexChanged(QString)), this, SLOT(setObstacles()));
                connect(m_ui->load_obstacles, SIGNAL(clicked()), this, SLOT(loadObstacles()));
                connect(m_ui->save_obstacles, SIGNAL(clicked()), this, SLOT(saveObstacles()));
                connect(m_ui->add_obstacles, SIGNAL(clicked()), this, SLOT(addObstacles()));
                connect(m_ui->delete_obstacles, SIGNAL(clicked()), this, SLOT(deleteObstacles()));

                connect(m_ui->robot_options, SIGNAL(currentIndexChanged(QString)), this, SLOT(setRobot()));
                */
            }

            void PlannerWidget::setRobot()
            {
                /*
                std::string robot_name = m_ui->robot_options->currentText().toStdString();
                */
            }

            void PlannerWidget::setObstacles()
            {
                // std::string obstacle_name = m_ui->obstacle_options->currentText().toStdString();
            }

            void PlannerWidget::saveObstacles()
            {
                /*
                std::string obstacle_name = m_ui->obstacle_options->currentText().toStdString();
                if (obstacle_name.size() != 0)
                {
                    m_settings->saveObstacles(obstacle_name);
                }
                */
            }

            void PlannerWidget::loadObstacles()
            {
                /*
                QString filepath = QFileDialog::getOpenFileName(this, "Load obstacles", QString(), "*.json");
                if (!filepath.isNull())
                {
                    m_settings->loadObstacles(filepath.toStdString());
                }
                */
            }

            void PlannerWidget::setTrajectory()
            {
                /*
                std::string trajectory_name = m_ui->trajectory_options->currentText().toStdString();
                if (trajectory_name.size() != 0)
                {
                    m_compute_thread->setTrajectory(trajectory_name);
                }
                */
            }

            void PlannerWidget::loadTrajectory()
            {
                /*
                QString filepath = QFileDialog::getOpenFileName(this, "Load trajectory", QString(), "*.json");
                if (!filepath.isNull())
                {
                    m_settings->loadTrajectory(filepath.toStdString());
                }
                */
            }

            void PlannerWidget::saveTrajectory()
            {
                /*
                std::string trajectory_name = m_ui->trajectory_options->currentText().toStdString();
                if (trajectory_name.size() != 0)
                {
                    m_settings->saveTrajectory(trajectory_name);
                }
                */
            }

        }
    }
}
