#include "path_planner_widget.hpp"

#include <QPaintEvent>
#include <QPainter>
#include <QPointF>

#include <units.hpp>
#include <point.hpp>
#include <rectangle.hpp>
#include <waypoint.hpp>

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
                , m_compute_thread(ComputeThread::getInstance())
                , m_animate(false)
                , m_speed_up(1.0)
                , m_display("Center")
            {
                m_timer.setInterval(100);
                m_timer.start();
                connect(m_compute_thread.get(), SIGNAL(newWaypoints()), this, SLOT(update()));
                connect(m_compute_thread.get(), SIGNAL(newPlan()), this, SLOT(update()));
                connect(&m_timer, SIGNAL(timeout()), this, SLOT(update()));
            }

            void PathPlannerWidget::setWorld(const geometry::Polygon& world)
            {
                m_world = world;
                if(m_animate)
                {
                    m_start.restart();
                }
                updateGeometry();
                update();
            }

            void PathPlannerWidget::setAnimate(bool animate)
            {
                m_animate = animate;
                if(animate)
                {
                    m_start.restart();
                }
                update();
            }

            void PathPlannerWidget::setSpeedUp(float speed_up)
            {
                m_speed_up = speed_up;
                m_start.restart();
                update();
            }

            void PathPlannerWidget::setDisplay(const QString& display)
            {
                m_display = display;
                update();
            }

            void PathPlannerWidget::paintEvent(QPaintEvent* event)
            {
                QPainter painter;
                painter.begin(this);
                painter.setRenderHint(QPainter::Antialiasing);
                painter.fillRect(event->rect(), Qt::white);

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

                painter.translate(-center.x()(), -center.y()());

                painter.save();

                drawWorld(painter);
                drawTrajectory(painter, scale.x()());
                drawWaypoints(painter, scale.x()());

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

                if (size != event->size())
                {
                    resize(size);
                }
                event->accept();
            }

            void PathPlannerWidget::drawWorld(QPainter& painter)
            {
                if(m_world.size())
                {
                    painter.setPen(QPen(Qt::black, 1));
                    painter.setBrush(Qt::blue);

                    QPainterPath path;

                    Point p = m_world[0];
                    path.moveTo(p.x()(), p.y()());
                    for(int i = 1, end = m_world.size(); i <= end; i++)
                    {
                        p = m_world[i % end];
                        path.lineTo(p.x()(), p.y()());
                    }
                    painter.drawPath(path);
                }
            }

            void PathPlannerWidget::drawWaypoints(QPainter& painter, double scale)
            {
                std::vector<navigation::pathplanner::Waypoint> waypoints = m_compute_thread->waypoints();

                if(waypoints.size())
                {
                    for(const navigation::pathplanner::Waypoint& waypoint: waypoints)
                    {
                        Point position = waypoint.position();
                        double x1 = position.x()();
                        double y1 = position.y()();
                        painter.setPen(QPen(Qt::black, 1));
                        painter.setBrush(Qt::yellow);
                        int radius = scale / 50;
                        painter.drawEllipse(x1 - radius / 2, y1 - radius / 2, radius, radius);
                        double x2 = x1 + radius * waypoint.tangent().x()();
                        double y2 = y1 + radius * waypoint.tangent().y()();
                        painter.setPen(QPen(Qt::red, scale / 250));
                        painter.setBrush(Qt::red);
                        painter.drawLine(x1, y1, x2, y2);
                    }
                }
            }

            void PathPlannerWidget::drawTrajectory(QPainter& painter, double scale)
            {
                std::vector< std::array<geometry::Point, 3> > trajectory;

                if(m_animate)
                {
                    Time current = m_start.elapsed() * units::ms * m_speed_up;
                    bool done = false;
                    trajectory = m_compute_thread->trajectory(0.10 * units::s, current, done);
                    if(done)
                    {
                        m_start.restart();
                    }
                }
                else
                {
                    trajectory = m_compute_thread->trajectory(0.10 * units::s);
                }
                if(trajectory.size())
                {
                    double width, length;

                    if(m_display == "Center")
                    {
                        painter.setPen(QPen(Qt::green, scale / 250));
                    }
                    else if(m_display == "Robot")
                    {
                        painter.setBrush(Qt::green);

                        width = m_compute_thread->width()();
                        length = m_compute_thread->length()();
                        if(m_animate)
                        {
                            Point last = trajectory.back()[1];
                            Point previous = trajectory[trajectory.size() - 2][1];
                            QPointF center(last.x()(), last.y()());
                            double rotation = atan(last - previous).to(units::radian);
                            QPolygonF polygon = createRect(center, rotation, width, length);

                            for(QPointF point : polygon)
                            {
                                if(!m_world.inside(Point(point.x(), point.y())))
                                {
                                    painter.setBrush(Qt::red);
                                    break;
                                }
                            }
                            painter.drawPolygon(polygon);
                            return;
                        }
                    }

                    QPointF previous_left(trajectory[0][0].x()(), trajectory[0][0].y()());
                    QPointF previous_center(trajectory[0][1].x()(), trajectory[0][1].y()());
                    QPointF previous_right(trajectory[0][2].x()(), trajectory[0][2].y()());

                    for(int i = 1, end = trajectory.size(); i < end; i++)
                    {
                        if(m_display == "Center")
                        {
                            QPointF center(trajectory[i][1].x()(), trajectory[i][1].y()());
                            painter.drawLine(previous_center, center);

                            previous_center = center;

                        }
                        else if(m_display == "Center & Sides")
                        {

                            painter.setPen(QPen(Qt::green, scale / 250));
                            QPointF center(trajectory[i][1].x()(), trajectory[i][1].y()());
                            painter.drawLine(previous_center, center);

                            painter.setPen(QPen(Qt::magenta, scale / 250));
                            QPointF left(trajectory[i][0].x()(), trajectory[i][0].y()());
                            painter.drawLine(previous_left, left);

                            QPointF right(trajectory[i][2].x()(), trajectory[i][2].y()());
                            painter.drawLine(previous_right, right);

                            previous_left = left;
                            previous_center = center;
                            previous_right = right;
                        }
                        else if(m_display == "Robot")
                        {
                            QPointF center(trajectory[i][1].x()(), trajectory[i][1].y()());
                            double rotation = atan(trajectory[i][1] - trajectory[i -1][1]).to(units::radian);
                            QPolygonF polygon = createRect(center, rotation, width, length);

                            painter.setBrush(Qt::green);
                            for(QPointF point : polygon)
                            {
                                if(!m_world.inside(Point(point.x(), point.y())))
                                {
                                    painter.setBrush(Qt::red);
                                    break;
                                }
                            }

                            painter.drawPolygon(polygon);
                            previous_center = center;
                        }
                    }
                }
            }

            QPolygonF PathPlannerWidget::createRect(const QPointF& center, double rotation, double width, double length)
            {
                if(width == 0 || length == 0)
                {
                    return QPolygonF();
                }

                QPointF back_left(-length / 2.0 * cos(rotation) + width / 2.0 * sin(rotation) + center.x(),
                                  -length / 2.0 * sin(rotation) - width / 2.0 * cos(rotation) + center.y());

                QPointF back_right(-length / 2.0 * cos(rotation) +  -width / 2.0 * sin(rotation) + center.x(),
                                   -length / 2.0 * sin(rotation) -  -width / 2.0 * cos(rotation) + center.y());

                QPointF front_right(length / 2.0 * cos(rotation) + -width / 2.0 * sin(rotation) + center.x(),
                                    length / 2.0 * sin(rotation) - -width / 2.0 * cos(rotation) + center.y());

                QPointF front_left(length / 2.0 * cos(rotation) + width / 2.0 * sin(rotation) + center.x(),
                                   length / 2.0 * sin(rotation) - width / 2.0 * cos(rotation) + center.y());

                QPolygonF polygon;
                polygon.append(back_left);
                polygon.append(back_right);
                polygon.append(front_right);
                polygon.append(front_left);
                return polygon;
            }
        }
    }
}
