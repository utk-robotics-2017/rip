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

                m_ui->x->setText(QString::number(m_x.to(units::in)));
                m_ui->y->setText(QString::number(m_y.to(units::in)));

                connect(m_ui->x, SIGNAL(currentTextChanged(QString)), this, SLOT(setX(QString)));
                connect(m_ui->y, SIGNAL(currentTextChanged(QString)), this, SLOT(setY(QString)));
            }

            PositionWidget::PositionWidget(const units::Distance& x, const units::Distance& y, QWidget* parent)
                : QWidget(parent)
                , m_x(x)
                , m_y(y)
                , m_ui(new Ui::PositionWidget)
            {
                m_ui->setupUi(this);

                m_ui->x->setText(QString::number(m_x.to(units::in)));
                m_ui->y->setText(QString::number(m_y.to(units::in)));

                connect(m_ui->x, SIGNAL(currentTextChanged(QString)), this, SLOT(setX(QString)));
                connect(m_ui->y, SIGNAL(currentTextChanged(QString)), this, SLOT(setY(QString)));
            }

            units::Distance PositionWidget::x() const
            {
                return m_x;
            }

            units::Distance PositionWidget::y() const
            {
                return m_y;
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
