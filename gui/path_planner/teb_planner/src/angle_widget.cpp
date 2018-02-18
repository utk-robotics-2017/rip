#include <teb_planner_gui/angle_widget.hpp>
#include "ui_angle_widget.h"

namespace rip
{
    namespace gui
    {
        namespace tebplanner
        {
            AngleWidget::AngleWidget(const units::Angle& theta, QWidget* parent)
                : QWidget(parent)
                , m_ui(new Ui::AngleWidget)
                , m_theta(theta)
            {
                m_ui->setupUi(this);

                m_ui->theta->setText(QString::number(m_theta.to(units::deg), 'f', 3));

                connect(m_ui->theta, SIGNAL(textEdited(QString)), this, SLOT(setTheta(QString)));
            }

            units::Angle AngleWidget::theta() const
            {
                return m_theta;
            }

            void AngleWidget::setTheta(const units::Angle& theta)
            {
                m_theta = theta;
                m_ui->theta->setText(QString::number(m_theta.to(units::deg), 'f', 3));
                emit updateAngle();
            }

            void AngleWidget::setTheta(const QString& text)
            {
                bool ok;
                double t_d = text.toDouble(&ok);
                if (ok)
                {
                    m_theta = t_d * units::deg;
                    emit updateAngle();
                }
            }
        }
    }
}
