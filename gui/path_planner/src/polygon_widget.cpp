#include "polygon_widget.hpp"

#include <QPainter>
#include <QResizeEvent>

namespace rip
{
    namespace gui
    {
        namespace pathplanner
        {
            PolygonWidget::PolygonWidget(QWidget* parent)
                : QWidget(parent)
            {
            }

            void PolygonWidget::updatePolygon(const geometry::Polygon& polygon)
            {
                m_polygon = polygon;
                updateGeometry();
                repaint();
            }

            void PolygonWidget::paintEvent(QPaintEvent* event)
            {
                QPainter painter;
                painter.begin(this);
                painter.setRenderHint(QPainter::Antialiasing);

                painter.fillRect(event->rect(), Qt::yellow);

                painter.translate(width() / 2.0, height() / 2.0);

                units::Distance min_x, max_x, min_y, max_y;
                geometry::Rectangle rect = m_polygon.boundingBox();
                geometry::Point center = rect.center();
                min_x = center.x() - (center.x() - rect.minX()) * 1.2;
                max_x = center.x() + (rect.maxX() - center.x()) * 1.2;
                min_y = center.y() - (center.y() - rect.minY()) * 1.2;
                max_y = center.y() + (rect.maxY() - center.x()) * 1.2;

                geometry::Point scale(max_x - min_x, max_y - min_y);
                painter.scale(width(), -height());
                painter.scale( 1.0 / scale.x()(), 1.0 / scale.y()());

                painter.translate(-center.x()(), -center.y()());

                painter.save();

                if(m_polygon.size())
                {
                    painter.setPen(QPen(Qt::black, 1));

                    painter.setBrush(Qt::white);

                    QPainterPath path;

                    geometry::Point p = m_polygon[0];
                    path.moveTo(p.x()(), p.y()());
                    for(int i = 1, end = m_polygon.size(); i <= end; i++)
                    {
                        p = m_polygon[i % end];
                        path.lineTo(p.x()(), p.y()());
                    }
                    painter.drawPath(path);
                }


                painter.restore();
                painter.end();

            }

            void PolygonWidget::resizeEvent(QResizeEvent* event)
            {
                int given_w = event->size().width();
                int given_h = event->size().height();

                int needed_w, needed_h;

                units::Distance min_x, max_x, min_y, max_y;

                geometry::Rectangle rect = m_polygon.boundingBox();
                geometry::Point center = rect.center();
                min_x = center.x() - (center.x() - rect.minX()) * 1.2;
                max_x = center.x() + (rect.maxX() - center.x()) * 1.2;
                min_y = center.y() - (center.y() - rect.minY()) * 1.2;
                max_y = center.y() + (rect.maxY() - center.x()) * 1.2;

                units::Distance desired_w = max_x - min_x;
                units::Distance desired_h = max_y - min_y;

                needed_h = roundf(given_w * desired_h() / desired_w());
                needed_w = roundf(given_h * desired_w() / desired_h());

                QSize size;
                if (needed_w < given_w)
                {
                    size = QSize(needed_w, given_h);
                }
                else
                {
                    size = QSize(given_w, needed_h);
                }

                if (size != event->size()) {
                    resize(size);
                }
                event->accept();
            }

        }
    }
}
