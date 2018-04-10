#ifndef POSITION_WIDGET_HPP
#define POSITION_WIDGET_HPP

#include <memory>

#include <QWidget>
#include <QString>

#include <units/units.hpp>
#include <geometry/point.hpp>

namespace Ui
{
    class PositionWidget;
}

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            namespace gui
            {
                class PositionWidget : public QWidget
                {
                    Q_OBJECT
                public:
                    explicit PositionWidget(const geometry::Point& point, QWidget* parent = nullptr);

                    explicit PositionWidget(const units::Distance& x, const units::Distance& y, QWidget* parent = nullptr);

                    units::Distance x() const;

                    void setX(const units::Distance& x);

                    units::Distance y() const;

                    void setY(const units::Distance& y);

                    geometry::Point position() const;

                    void setPosition(const geometry::Point& p);
                signals:
                    void updatePosition();

                private slots:
                    void setX(const QString& x);

                    void setY(const QString& y);

                private:
                    std::shared_ptr<Ui::PositionWidget> m_ui;
                    units::Distance m_x;
                    units::Distance m_y;
                };
            }
        }
    }
}

#endif // POSITION_WIDGET_HPP
