#include "motor_settings_widget.hpp"
#include "ui_motor_settings_widget.h"

#include <QInputDialog>

namespace rip
{
    namespace gui
    {
        namespace motorcalc
        {
            MotorSettingsWidget::MotorSettingsWidget(QWidget* parent)
                : QWidget(parent)
                , m_ui(new Ui::MotorSettingsWidget)
                , m_settings(Settings::getInstance())
            {
                m_ui->setupUi(this);

                connect(m_ui->options, SIGNAL(currentIndexChanged(QString)), this, SLOT(updateMotor(QString)));

                connect(m_ui->free_speed, SIGNAL(textChanged(QString)), this, SLOT(updateFreeSpeed(QString)));
                connect(m_ui->stall_torque, SIGNAL(textChanged(QString)), this, SLOT(updateStallTorque(QString)));
                connect(m_ui->stall_current, SIGNAL(textChanged(QString)), this, SLOT(updateStallCurrent(QString)));
                connect(m_ui->free_current, SIGNAL(textChanged(QString)), this, SLOT(updateFreeCurrent(QString)));
                connect(m_ui->voltage, SIGNAL(textChanged(QString)), this, SLOT(updateVoltage(QString)));

                connect(m_ui->add, SIGNAL(clicked(bool)), this, SLOT(addMotor()));

                m_ui->free_speed->setEnabled(false);
                m_ui->stall_torque->setEnabled(false);
                m_ui->stall_current->setEnabled(false);
                m_ui->free_current->setEnabled(false);
                m_ui->voltage->setEnabled(false);

                m_chart = new QtCharts::QChart();
                m_chart->legend()->setAlignment(Qt::AlignTop);
                m_chart->setTitle("Motor Characteristics");

                m_chart_view = new QtCharts::QChartView(m_chart);
                m_chart_view->setRenderHint(QPainter::Antialiasing);
                m_ui->verticalLayout->addWidget(m_chart_view);
            }

            void MotorSettingsWidget::setOptions(const std::vector<std::string>& options)
            {
                if (options.size())
                {
                    QStringList list;
                    for (const std::string& item : options)
                    {
                        list << QString::fromStdString(item);
                    }
                    m_ui->options->addItems(list);
                    updateMotor(QString::fromStdString(options.front()));
                }
            }

            std::string MotorSettingsWidget::motor() const
            {
                if (m_current_motor)
                {
                    return m_current_motor->name();
                }
                return "";
            }

            void MotorSettingsWidget::updateMotor(QString name)
            {
                m_current_motor = m_settings->motor(name.toStdString());
                if (m_current_motor)
                {
                    m_ui->free_speed->setEnabled(true);
                    m_ui->stall_torque->setEnabled(true);
                    m_ui->stall_current->setEnabled(true);
                    m_ui->free_current->setEnabled(true);
                    m_ui->voltage->setEnabled(true);

                    units::AngularVelocity free_speed = m_current_motor->get<units::AngularVelocity>("free_speed");
                    units::Torque stall_torque = m_current_motor->get<units::Torque>("stall_torque");
                    units::Current stall_current = m_current_motor->get<units::Current>("stall_current");
                    units::Current free_current = m_current_motor->get<units::Current>("free_current");
                    units::Voltage voltage = m_current_motor->get<units::Voltage>("voltage");

                    m_ui->free_speed->setText(QString::number(free_speed.to(units::rev / units::minute)));
                    m_ui->stall_torque->setText(QString::number(stall_torque.to(units::oz * units::in)));
                    m_ui->stall_current->setText(QString::number(stall_current.to(units::A)));
                    m_ui->free_current->setText(QString::number(free_current.to(units::A)));
                    m_ui->voltage->setText(QString::number(m_current_motor->get<units::Voltage>("voltage").to(units::V)));

                    updateCharts(free_speed, stall_torque, stall_current, free_current, voltage);
                }
                else
                {
                    m_ui->free_speed->setEnabled(false);
                    m_ui->stall_torque->setEnabled(false);
                    m_ui->stall_current->setEnabled(false);
                    m_ui->free_current->setEnabled(false);
                    m_ui->voltage->setEnabled(false);
                }
            }

            void MotorSettingsWidget::updateCharts()
            {
                units::AngularVelocity free_speed = m_current_motor->get<units::AngularVelocity>("free_speed");
                units::Torque stall_torque = m_current_motor->get<units::Torque>("stall_torque");
                units::Current stall_current = m_current_motor->get<units::Current>("stall_current");
                units::Current free_current = m_current_motor->get<units::Current>("free_current");
                units::Voltage voltage = m_current_motor->get<units::Voltage>("voltage");

                updateCharts(free_speed, stall_torque, stall_current, free_current, voltage);
            }

            void MotorSettingsWidget::updateCharts(const units::AngularVelocity& free_speed, const units::Torque& stall_torque, const units::Current& stall_current, const units::Current free_current, const units::Voltage& voltage)
            {
                m_chart->removeAllSeries();
                if(m_x_axis)
                {
                    m_chart->removeAxis(m_x_axis);
                }
                if(m_speed_axis)
                {
                    m_chart->removeAxis(m_speed_axis);
                }
                if(m_power_axis)
                {
                    m_chart->removeAxis(m_power_axis);
                }
                if(m_efficiency_axis)
                {
                    m_chart->removeAxis(m_efficiency_axis);
                }
                if(m_current_axis)
                {
                    m_chart->removeAxis(m_current_axis);
                }

                if(free_speed == 0.0 ||
                   stall_torque == 0.0 ||
                   stall_current == 0.0 ||
                   voltage == 0.0)
                {
                    return;
                }

                m_speed = new QtCharts::QLineSeries;
                m_speed->setName("Speed");

                m_power = new QtCharts::QLineSeries;
                m_power->setName("Power");

                m_efficiency = new QtCharts::QLineSeries;
                m_efficiency->setName("Efficiency");

                m_current = new QtCharts::QLineSeries;
                m_current->setName("Current");

                // 100 data points
                units::Torque inc = stall_torque / 100.0;

                for(units::Torque torque = 0; torque <= stall_torque; torque += inc)
                {

                    units::AngularVelocity speed = (stall_torque - torque) * free_speed / stall_torque;
                    m_speed->append(QPointF(torque.to(units::oz * units::in), speed.to(units::rev / units::minute)));

                    units::Power power = speed() * torque();
                    m_power->append(QPointF(torque.to(units::oz * units::in), power.to(units::W)));

                    units::Current current = (stall_current - free_current) * (torque) / (stall_torque) +free_current;
                    m_current->append(QPointF(torque.to(units::oz * units::in), current.to(units::A)));

                    units::Power power_in = voltage() * current();
                    m_efficiency->append(QPointF(torque.to(units::oz * units::in), power() * 100 / power_in() ));
                }


                m_x_axis = new QtCharts::QValueAxis;
                m_x_axis->setTickCount(10);
                m_x_axis->setTitleText("Load (oz*in)");
                m_chart->addAxis(m_x_axis, Qt::AlignBottom);

                m_chart->addSeries(m_speed);

                m_speed_axis = new QtCharts::QValueAxis;
                m_speed_axis->setLinePenColor(m_speed->pen().color());
                m_speed_axis->setTickCount(11);
                m_speed_axis->setTitleText("Speed (RPM)");
                m_chart->addAxis(m_speed_axis, Qt::AlignLeft);
                m_speed->attachAxis(m_x_axis);
                m_speed->attachAxis(m_speed_axis);

                m_chart->addSeries(m_power);
                m_power_axis = new QtCharts::QValueAxis;
                m_power_axis->setLinePenColor(m_power->pen().color());
                m_power_axis->setTickCount(11);
                m_power_axis->setTitleText("Power (W)");
                m_chart->addAxis(m_power_axis, Qt::AlignRight);
                m_power->attachAxis(m_x_axis);
                m_power->attachAxis(m_power_axis);

                m_chart->addSeries(m_efficiency);
                m_efficiency_axis = new QtCharts::QValueAxis;
                m_efficiency_axis->setLinePenColor(m_efficiency->pen().color());
                m_efficiency_axis->setTickCount(11);
                m_efficiency_axis->setTitleText("Efficiency (%)");
                m_efficiency_axis->setMin(0);
                m_efficiency_axis->setMax(100);
                m_chart->addAxis(m_efficiency_axis, Qt::AlignLeft);
                m_efficiency->attachAxis(m_x_axis);
                m_efficiency->attachAxis(m_efficiency_axis);

                m_chart->addSeries(m_current);
                m_current_axis = new QtCharts::QValueAxis;
                m_current_axis->setLinePenColor(m_current->pen().color());
                m_current_axis->setTickCount(11);
                m_current_axis->setTitleText("Current (A)");
                m_chart->addAxis(m_current_axis, Qt::AlignRight);
                m_current->attachAxis(m_x_axis);
                m_current->attachAxis(m_current_axis);
            }

            void MotorSettingsWidget::addMotor()
            {
                bool ok;
                QString name = QInputDialog::getText(this, tr("Add Motor"), tr("Name:"), QLineEdit::Normal, "Default", &ok);
                if (ok && !name.isEmpty())
                {
                    m_settings->addMotor(name.toStdString());
                    m_ui->options->addItem(name);
                    updateCharts();
                }
            }

            void MotorSettingsWidget::updateFreeSpeed(const QString& value)
            {
                bool ok = false;
                double d = value.toDouble(&ok);
                if (ok)
                {
                    units::AngularVelocity free_speed = d * units::rev / units::minute;
                    m_current_motor->set<units::AngularVelocity>("free_speed", free_speed);
                    updateCharts();
                }
            }

            void MotorSettingsWidget::updateStallTorque(const QString& value)
            {
                bool ok = false;
                double d = value.toDouble(&ok);
                if (ok)
                {
                    units::Torque stall_torque = d * units::oz * units::in;
                    m_current_motor->set<units::Torque>("stall_torque", stall_torque);
                    updateCharts();
                }
            }

            void MotorSettingsWidget::updateStallCurrent(const QString& value)
            {
                bool ok = false;
                double d = value.toDouble(&ok);
                if (ok)
                {
                    units::Current stall_current = d * units::A;
                    m_current_motor->set<units::Current>("stall_current", stall_current);
                    updateCharts();
                }
            }

            void MotorSettingsWidget::updateFreeCurrent(const QString& value)
            {
                bool ok = false;
                double d = value.toDouble(&ok);
                if (ok)
                {
                    units::Current free_current = d * units::A;
                    m_current_motor->set<units::Current>("free_current", free_current);
                    updateCharts();
                }
            }

            void MotorSettingsWidget::updateVoltage(const QString& value)
            {
                bool ok = false;
                double d = value.toDouble(&ok);
                if (ok)
                {
                    units::Voltage voltage = d * units::V;
                    m_current_motor->set<units::Voltage>("voltage", voltage);
                    updateCharts();
                }
            }
        }
    }
}
