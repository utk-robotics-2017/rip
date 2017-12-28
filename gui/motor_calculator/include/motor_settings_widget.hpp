#ifndef MOTOR_SETTINGS_WIDGET_HPP
#define MOTOR_SETTINGS_WIDGET_HPP

#include <vector>
#include <memory>

#include <QWidget>
#include <QString>

#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>

#include "settings.hpp"
#include <units.hpp>

namespace Ui
{
    class MotorSettingsWidget;
}

namespace rip
{
    namespace gui
    {
        namespace motorcalc
        {
            class MotorSettingsWidget : public QWidget
            {
                Q_OBJECT
            public:
                explicit MotorSettingsWidget(QWidget* parent = nullptr);

                void setOptions(const std::vector<std::string>& options);

                std::string motor() const;

            public slots:
                void updateMotor(QString name);

                void addMotor();

                void updateFreeSpeed(const QString& value);

                void updateStallTorque(const QString& value);

                void updateStallCurrent(const QString& value);

                void updateFreeCurrent(const QString& value);

                void updateVoltage(const QString& value);

            private:
                void updateCharts();
                void updateCharts(const units::AngularVelocity& free_speed, const units::Torque& stall_torque, const units::Current& stall_current, const units::Current free_current, const units::Voltage& voltage);

                Ui::MotorSettingsWidget* m_ui;

                std::shared_ptr<misc::SettingsBase> m_current_motor;
                std::shared_ptr<Settings> m_settings;

                QtCharts::QChartView* m_chart_view;
                QtCharts::QChart* m_chart;
                QtCharts::QValueAxis* m_x_axis;

                QtCharts::QLineSeries* m_speed;
                QtCharts::QValueAxis* m_speed_axis;

                QtCharts::QLineSeries* m_power;
                QtCharts::QValueAxis* m_power_axis;

                QtCharts::QLineSeries* m_efficiency;
                QtCharts::QValueAxis* m_efficiency_axis;

                QtCharts::QLineSeries* m_current;
                QtCharts::QValueAxis* m_current_axis;
            };
        }
    }
}

#endif // MOTOR_SETTINGS_WIDGET_HPP
