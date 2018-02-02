#include <teb_planner_gui/robot_outer_widget.hpp>
#include "ui_robot_outer_widget.h"

#include <QInputDialog>
#include <QLineEdit>

namespace rip
{
    namespace gui
    {
        namespace tebplanner
        {

            RobotOuterWidget::RobotOuterWidget(QWidget* parent)
                : QWidget(parent)
                , m_ui(new Ui::RobotOuterWidget)
                , m_settings(Settings::getInstance())
            {
                m_ui->setupUi(this);

                connect(m_ui->add, SIGNAL(clicked(bool)), this, SLOT(addRobot()));
                connect(m_ui->options, SIGNAL(currentIndexChanged(QString)), this, SLOT(setRobot(QString)));
                connect(m_ui->remove, SIGNAL(clicked(bool)), this, SLOT(removeRobot()));
            }

            void RobotOuterWidget::setOptions()
            {
                QList<QString> names;
                for(const std::string& name : m_settings->getRobotNames())
                {
                    names << QString(name.c_str());
                }
                m_ui->options->addItems(names);
            }

            void RobotOuterWidget::addRobot()
            {
                bool ok;
                QString name = QInputDialog::getText(this, tr("Add Robot"), tr("Name:"), QLineEdit::Normal, "Default", &ok);
                if (ok && !name.isEmpty())
                {
                    std::shared_ptr<navigation::tebplanner::PolygonRobotFootprint> robot = m_settings->addRobot(name.toStdString());

                    std::vector< geometry::Point > points;
                    units::Angle step = 90 * units::deg;
                    for (int i = 0; i < 4; i++)
                    {
                        points.emplace_back(units::cos(i * step + 45 * units::deg), units::sin(i * step + 45 * units::deg));
                        points.back() *= units::in();
                    }
                    robot->setPolygon(points);

                    m_ui->options->addItem(name);
                    m_name = name.toStdString();
                    m_ui->widget->setRobot(robot);
                }
            }

            void RobotOuterWidget::setRobot(const QString& text)
            {
                std::string name = text.toStdString();
                std::shared_ptr<navigation::tebplanner::PolygonRobotFootprint> robot = m_settings->robot(name);
                m_ui->widget->setRobot(robot);
            }

            void RobotOuterWidget::removeRobot()
            {
                int index = m_ui->options->currentIndex();
                if (index >= 0)
                {
                    m_settings->removeRobot(m_name);
                    m_ui->options->removeItem(index);
                }
            }

        }
    }
}
