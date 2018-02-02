#include <teb_planner_gui/position_widget.hpp>

#include "ui_position_widget.h"

namespace rip
{
    namespace gui
    {
        namespace tebplanner
        {
            PositionWidget::PositionWidget(const geometry::Point& p, QWidget* parent)
                : QWidget(parent)
                , m_x(p.x())
                , m_y(p.y())
                , m_ui(new Ui::PositionWidget)
            {
                m_ui->setupUi(this);

                m_ui->x->setText(QString::number(m_x.to(units::in), 'f', 2));
                m_ui->y->setText(QString::number(m_y.to(units::in), 'f', 2));

                connect(m_ui->x, SIGNAL(textChanged(QString)), this, SLOT(setX(QString)));
                connect(m_ui->y, SIGNAL(textChanged(QString)), this, SLOT(setY(QString)));
            }

            PositionWidget::PositionWidget(const units::Distance& x, const units::Distance& y, QWidget* parent)
                : QWidget(parent)
                , m_x(x)
                , m_y(y)
                , m_ui(new Ui::PositionWidget)
            {
                m_ui->setupUi(this);

                m_ui->x->setText(QString::number(m_x.to(units::in), 'f', 3));
                m_ui->y->setText(QString::number(m_y.to(units::in), 'f', 3));

                connect(m_ui->x, SIGNAL(textChanged(QString)), this, SLOT(setX(QString)));
                connect(m_ui->y, SIGNAL(textChanged(QString)), this, SLOT(setY(QString)));
            }

            units::Distance PositionWidget::x() const
            {
                return m_x;
            }

            void PositionWidget::setX(const units::Distance& x)
            {
                m_x = x;
                m_ui->x->setText(QString::number(m_x.to(units::in), 'f', 3));
            }

            units::Distance PositionWidget::y() const
            {
                return m_y;
            }

            void PositionWidget::setY(const units::Distance& y)
            {
                m_y = y;
                m_ui->y->setText(QString::number(m_y.to(units::in), 'f', 3));
            }

            geometry::Point PositionWidget::position() const
            {
                return geometry::Point(m_x, m_y);
            }

            void PositionWidget::setPosition(const geometry::Point& p)
            {
                setX(p.x());
                setY(p.y());
            }

            void PositionWidget::setX(const QString& x)
            {
                bool ok;
                double x_d = x.toDouble(&ok);
                if (ok)
                {
                    m_x = x_d * units::in;
                    emit updatePosition();
                }
            }

            void PositionWidget::setY(const QString& y)
            {
                bool ok;
                double y_d = y.toDouble(&ok);
                if (ok)
                {
                    m_y = y_d * units::in;
                    emit updatePosition();
                }
            }
        }
    }
}
