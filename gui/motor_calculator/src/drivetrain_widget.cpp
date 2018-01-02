#include "drivetrain_widget.hpp"
#include "ui_drivetrain_widget.h"

#include "motor_list_widget_item.hpp"

namespace rip
{
    namespace gui
    {
        namespace motorcalc
        {

            DrivetrainWidget::DrivetrainWidget(QWidget* parent)
                : QWidget(parent)
                , m_ui(new Ui::DrivetrainWidget)
            {
                m_ui->setupUi(this);

                connect(m_ui->num_motors, SIGNAL(textChanged(QString)), this, SLOT(updateNumberOfMotor(QString)));
                connect(m_ui->weight, SIGNAL(textChanged(QString)), this, SLOT(updateOther()));
                connect(m_ui->radius, SIGNAL(textChanged(QString)), this, SLOT(updateOther()));
            }

            DrivetrainWidget::~DrivetrainWidget()
            {
                delete m_ui;
            }

            void DrivetrainWidget::updateNumberOfMotor(const QString& value)
            {
                bool ok = false;
                int num_motors = value.toInt(&ok);
                if (ok)
                {
                    int count = m_ui->motors->count();
                    if (num_motors > count)
                    {
                        // Increase the number of rows
                        for (int i = count; i < num_motors; i++)
                        {
                            MotorListWidgetItem* mlwi = new MotorListWidgetItem(m_ui->motors);
                            mlwi->updateMotorOptions();
                            mlwi->resize(m_ui->motors->size());
                            mlwi->show();
                            QListWidgetItem* lwi = new QListWidgetItem(m_ui->motors);
                            lwi->setSizeHint(mlwi->sizeHint());
                            m_ui->motors->addItem(lwi);
                            m_ui->motors->setItemWidget(lwi, mlwi);
                            connect(mlwi, SIGNAL(modified()), this, SLOT(updateSpeed()));
                        }
                        updateSpeed();
                    }
                    else if (num_motors < count)
                    {
                        // Decrease number of rows
                        for (int i = count - 1; i >= num_motors; i--)
                        {
                            QListWidgetItem* lwi = m_ui->motors->item(i);
                            delete m_ui->motors->itemWidget(lwi);
                            delete m_ui->motors->takeItem(i);
                        }
                        updateSpeed();
                    }

                }

            }

            void DrivetrainWidget::updateOther()
            {

            }

            void DrivetrainWidget::updateSpeed()
            {
                //std::shared_ptr<SettingsBase> motor = m_uui
            }
        }
    }
}
