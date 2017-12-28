#ifndef DRIVETRAIN_WIDGET_HPP
#define DRIVETRAIN_WIDGET_HPP

#include <QWidget>

namespace Ui
{
    class DrivetrainWidget;
}

namespace rip
{
    namespace gui
    {
        namespace motorcalc
        {
            class DrivetrainWidget : public QWidget
            {
                Q_OBJECT
            public:
                explicit DrivetrainWidget(QWidget* parent = nullptr);

                ~DrivetrainWidget();

            private slots:
                void updateNumberOfMotor(const QString& value);
                void updateOther();
                void updateSpeed();

            private:
                Ui::DrivetrainWidget* m_ui;
            };

        }
    }
}

#endif // DRIVETRAIN_WIDGET_HPP
