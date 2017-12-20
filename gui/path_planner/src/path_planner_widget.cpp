#include "path_planner_widget.hpp"

#include <QPaintEvent>
#include <QPainter>

#include <units.hpp>
#include <point.hpp>
#include <rectangle.hpp>



namespace rip
{
    using Rectangle = geometry::Rectangle;
    using Distance = units::Distance;
    using Point = geometry::Point;

    namespace gui
    {
        namespace pathplanner
        {
            PathPlannerWidget::PathPlannerWidget(QWidget* parent)
                : QWidget(parent)
            {
            }

            void PathPlannerWidget::setWorld(const geometry::PolygonList& world)
            {
                m_world = world;
            }

            void PathPlannerWidget::paintEvent(QPaintEvent* event)
            {
                QPainter painter;
                painter.begin(this);
                painter.setRenderHint(QPainter::Antialiasing);

                painter.translate(width() / 2.0, height() / 2.0);

                Distance min_x, max_x, min_y, max_y;
                Rectangle rect = m_world.boundingBox();
                Point center = rect.center();
                min_x = center.x() - (center.x() - rect.minX()) * 1.2;
                max_x = center.x() + (rect.maxX() - center.x()) * 1.2;
                min_y = center.y() - (center.y() - rect.minY()) * 1.2;
                max_y = center.y() + (rect.maxY() - center.x()) * 1.2;

                Point scale(max_x - min_x, max_y - min_y);
                painter.scale(width(), -height());
                painter.scale( 1.0 / scale.x()(), 1.0 / scale.y()());

                painter.save();

                painter.fillRect(QRectF(min_x(), max_y(), (max_x - min_x)(), (max_y - min_y)()), Qt::black);

                drawWorld(painter);
                drawTrajectory(painter);
                drawWaypoints(painter);

                painter.restore();
                painter.end();


            }

            void PathPlannerWidget::resizeEvent(QResizeEvent* event)
            {
                int given_w = event->size().width();
                int given_h = event->size().height();

                int needed_w, needed_h;

                Distance min_x, max_x, min_y, max_y;

                Rectangle rect = m_world.boundingBox();
                Point center = rect.center();
                min_x = center.x() - (center.x() - rect.minX()) * 1.2;
                max_x = center.x() + (rect.maxX() - center.x()) * 1.2;
                min_y = center.y() - (center.y() - rect.minY()) * 1.2;
                max_y = center.y() + (rect.maxY() - center.x()) * 1.2;

                Distance desired_w = max_x - min_x;
                Distance desired_h = max_y - min_y;

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

            void PathPlannerWidget::drawWorld(QPainter& painter)
            {
                for(const geometry::Polygon& polygon: m_world)
                {
                    if(polygon.size())
                    {
                        painter.setPen(QPen(Qt::black, 3000));
                        if(polygon.exterior())
                        {
                            painter.setBrush(Qt::blue);
                        }
                        else
                        {
                            painter.setBrush(Qt::red);
                        }

                        QPainterPath path;

                        Point p = polygon[0];
                        path.moveTo(p.x()(), p.y()());
                        for(int i = 1, end = polygon.size(); i <= end; i++)
                        {
                            p = polygon[i % end];
                            path.lineTo(p.x()(), p.y()());
                        }
                        painter.drawPath(path);
                    }
                }
            }

            void PathPlannerWidget::drawWaypoints(QPainter& painter)
            {

            }

            void PathPlannerWidget::drawTrajectory(QPainter& painter)
            {

            }
        }
    }
}
