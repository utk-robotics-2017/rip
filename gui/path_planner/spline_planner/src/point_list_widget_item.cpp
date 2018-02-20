#include <spline_planner_gui/point_list_widget_item.hpp>
#include "ui_point_list_widget_item.h"

namespace rip
{
    namespace gui
    {
        namespace splineplanner
        {

            PointListWidgetItem::PointListWidgetItem(QWidget* parent)
                : QWidget(parent)
                , m_ui(new Ui::PointListWidgetItem)
            {
                m_ui->setupUi(this);

                connect(m_ui->x, SIGNAL(textChanged(QString)), this, SLOT(updatedText()));
                connect(m_ui->y, SIGNAL(textChanged(QString)), this, SLOT(updatedText()));
            }

            PointListWidgetItem::~PointListWidgetItem()
            {
                delete m_ui;
            }

            double PointListWidgetItem::x() const
            {
                bool ok = false;
                double d = m_ui->x->text().toDouble(&ok);
                if(ok)
                {
                    return d;
                }
                return 0.0;
            }

            void PointListWidgetItem::setX(double x)
            {
                m_ui->x->setText(QString::number(x));
            }

            double PointListWidgetItem::y() const
            {
                bool ok = false;
                double d = m_ui->y->text().toDouble(&ok);
                if(ok)
                {
                    return d;
                }
                return 0.0;
            }

            void PointListWidgetItem::setY(double y)
            {
                m_ui->y->setText(QString::number(y));
            }

            void PointListWidgetItem::updatedText()
            {
                // Only update if text is valid number
                bool ok = false;
                m_ui->x->text().toDouble(&ok);
                if(ok)
                {
                    m_ui->y->text().toDouble(&ok);
                    if(ok)
                    {
                        emit modified();
                    }
                }
            }

        } // pathplanner
    } // gui
} // rip
