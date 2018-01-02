#ifndef MOTOR_LIST_WIDGET_ITEM_HPP
#define MOTOR_LIST_WIDGET_ITEM_HPP

#include <memory>

#include <QWidget>

#include "settings.hpp"

namespace Ui
{
    class MotorListWidgetItem;
}

namespace rip
{
    namespace gui
    {
        namespace motorcalc
        {
            class MotorListWidgetItem : public QWidget
            {
                Q_OBJECT
            public:
                explicit MotorListWidgetItem(QWidget* parent = nullptr);

                ~MotorListWidgetItem();

                void setOptions(const std::vector<std::string>& options);

                std::string motor() const;

                double gearRatio() const;

            signals:
                void modified();

            public slots:
                void updateMotorOptions();

            public:
                QSize minimumSizeHint() const override;
                QSize sizeHint() const override;

            private slots:
                void updateMotor(const QString& motor);

                void updateNumberOfGears(const QString& value);

                void updateGears();

            private:
                Ui::MotorListWidgetItem* m_ui;
                std::shared_ptr<Settings> m_settings;
            };
        }
    }
}

#endif // MOTOR_LIST_WIDGET_ITEM_HPP
