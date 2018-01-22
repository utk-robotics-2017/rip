#include <teb_planner_gui/robot_settings_widget.hpp>
#include "ui_robot_settings_widget.h"

namespace rip
{
    namespace gui
    {
        namespace tebplanner
        {
            RobotSettingsWidget::RobotSettingsWidget(QWidget* parent)
                : QWidget(parent)
                , m_ui(new Ui::RobotSettingsWidget)
            {
                m_ui->setupUi(this);

                // todo maps and connections
            }

            void RobotSettingsWidget::updateDistance()
            {

            }

            void RobotSettingsWidget::updateVelocity()
            {

            }

            void RobotSettingsWidget::updateAcceleration()
            {

            }

            void RobotSettingsWidget::updateDouble()
            {

            }

            void RobotSettingsWidget::updateInteger()
            {

            }

            void RobotSettingsWidget::updateBool()
            {

            }
        }
    }
}
