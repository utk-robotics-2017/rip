#include "motor_list_widget_item.hpp"
#include "ui_motor_list_widget_item.h"

#include "gear_list_widget_item.hpp"

namespace rip
{
    namespace gui
    {
        namespace motorcalc
        {

            MotorListWidgetItem::MotorListWidgetItem(QWidget* parent)
                : QWidget(parent)
                , m_ui(new Ui::MotorListWidgetItem)
                , m_settings(Settings::getInstance())
            {
                m_ui->setupUi(this);

                connect(m_ui->motor, SIGNAL(currentIndexChanged(QString)), this, SLOT(updateMotor(QString)));
                connect(m_ui->num_gears, SIGNAL(textChanged(QString)), this, SLOT(updateNumberOfGears(QString)));
                connect(m_settings.get(), SIGNAL(newMotor()), this, SLOT(updateMotorOptions()));
            }

            MotorListWidgetItem::~MotorListWidgetItem()
            {
                delete m_ui;
            }

            void MotorListWidgetItem::setOptions(const std::vector<std::string>& options)
            {
                if (options.size())
                {
                    QStringList list;
                    for (const std::string& item : options)
                    {
                        list << QString::fromStdString(item);
                    }
                    m_ui->motor->addItems(list);
                }
            }

            std::string MotorListWidgetItem::motor() const
            {
                return m_ui->motor->currentText().toStdString();
            }

            double MotorListWidgetItem::gearRatio() const
            {
                double ratio = 1;
                int count = m_ui->gears->count();
                for (int i = 0; i < count; i++)
                {
                    QListWidgetItem* lwi = m_ui->gears->item(i);
                    GearListWidgetItem* glwi = static_cast<GearListWidgetItem*>(m_ui->gears->itemWidget(lwi));
                    ratio *= glwi->ratio();
                }
                return ratio;
            }

            void MotorListWidgetItem::updateMotor(const QString& motor)
            {
                emit modified();
            }

            void MotorListWidgetItem::updateNumberOfGears(const QString& value)
            {
                bool ok = false;
                int num_gears = value.toInt(&ok);
                if (ok)
                {
                    int count = m_ui->gears->count();
                    if (num_gears > count)
                    {
                        // Increase the number of rows
                        for (int i = count; i < num_gears; i++)
                        {
                            GearListWidgetItem* glwi = new GearListWidgetItem(m_ui->gears);
                            glwi->resize(m_ui->gears->size());
                            glwi->show();
                            QListWidgetItem* lwi = new QListWidgetItem(m_ui->gears);
                            lwi->setSizeHint(glwi->sizeHint());
                            m_ui->gears->addItem(lwi);
                            m_ui->gears->setItemWidget(lwi, glwi);
                            connect(glwi, SIGNAL(modified()), this, SLOT(updateGears()));
                        }
                    }
                    else if (num_gears < count)
                    {
                        // Decrease number of rows
                        for (int i = count - 1; i >= num_gears; i--)
                        {
                            QListWidgetItem* lwi = m_ui->gears->item(i);
                            delete m_ui->gears->itemWidget(lwi);
                            delete m_ui->gears->takeItem(i);
                        }
                    }

                    emit modified();
                }
            }

            void MotorListWidgetItem::updateGears()
            {
                emit modified();
            }

            void MotorListWidgetItem::updateMotorOptions()
            {
                QString current_option = m_ui->motor->currentText();
                m_ui->motor->clear();
                std::vector<std::string> motor_names = m_settings->motorNames();
                setOptions(motor_names);

                for(const std::string& option: motor_names)
                {
                    if(option == current_option.toStdString())
                    {
                        m_ui->motor->setCurrentText(current_option);
                        break;
                    }
                }

            }

            QSize MotorListWidgetItem::minimumSizeHint() const
            {
                return QSize(0, QWidget::minimumSizeHint().height());
            }

            QSize MotorListWidgetItem::sizeHint() const
            {
                return QSize(0, QWidget::sizeHint().height());
            }

        }
    }
}
