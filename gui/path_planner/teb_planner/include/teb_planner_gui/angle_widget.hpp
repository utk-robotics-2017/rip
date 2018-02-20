#ifndef ANGLE_WIDGET_HPP
#define ANGLE_WIDGET_HPP

#include <QWidget>

#include <units/units.hpp>

namespace Ui
{
    class AngleWidget;
}

namespace rip
{
    namespace gui
    {
        namespace tebplanner
        {
            class AngleWidget : public QWidget
            {
                Q_OBJECT
            public:
                explicit AngleWidget(const units::Angle& theta, QWidget* parent = nullptr);

                units::Angle theta() const;

                void setTheta(const units::Angle& theta);

            private slots:
                void setTheta(const QString& text);

            signals:
                void updateAngle();

            private:
                std::shared_ptr< Ui::AngleWidget > m_ui;
                units::Angle m_theta;
            };
        }
    }
}

#endif // ANGLE_WIDGET_HPP
