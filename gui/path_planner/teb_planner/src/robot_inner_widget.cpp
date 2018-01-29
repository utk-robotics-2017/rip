#include <teb_planner_gui/robot_inner_widget.hpp>

#include <QMenu>
#include <QPainter>
#include <QPaintEvent>
#include <QResizeEvent>
#include <QPen>
#include <QInputDialog>

namespace rip
{
    namespace gui
    {
        namespace tebplanner
        {

            RobotInnerWidget::RobotInnerWidget(QWidget* parent)
                : QWidget(parent)
                , m_settings(Settings::getInstance())
                , m_position_widget(nullptr)
                , m_robot(nullptr)
                , m_bounding_box(-1 * units::in, 1 * units::in, -1 * units::in, 1 * units::in)
            {

            }

            void RobotInnerWidget::setRobot(std::shared_ptr<navigation::PolygonRobotFootprintModel> robot)
            {
                m_robot = robot;
                update();
            }

            void RobotInnerWidget::mousePressEvent(QMouseEvent* event)
            {
                if (!m_robot)
                {
                    return;
                }

                if (event->button() == Qt::RightButton)
                {
                    showContextMenu(event->pos());
                    return;
                }

                QPoint pos = event->pos();

                double s;
                QMatrix transform = getTransform(&s);

                // If click out side the bounding box then short circuit
                QMatrix inverse_transform = transform.inverted();
                QPoint rp =  pos * inverse_transform;
                geometry::Point real_point(rp.x(), rp.y());
                if (m_bounding_box.clip(real_point) != real_point)
                {
                    return;
                }

                s /= 250;
                geometry::Polygon poly = m_robot->polygon();
                for (int i = 0; i < poly.size(); i++)
                {
                    if (checkPointClick(poly[i], transform, pos, i))
                    {
                        return;
                    }
                }

                m_selected = false;
            }

            bool RobotInnerWidget::checkPointClick(const geometry::Point& point, const QMatrix& transform, const QPoint& mouse_pos, int index, unsigned int pixel_threshold)
            {
                QPoint screen_point(point.x()(), point.y()());
                screen_point = screen_point * transform;
                if ((screen_point - mouse_pos).manhattanLength() < pixel_threshold)
                {
                    m_selected = true;
                    m_selected_index = index;

                    m_position_widget = std::unique_ptr<PositionWidget>(new PositionWidget(point, this));
                    m_position_widget->move(QPoint(screen_point.x() - 90, screen_point.y() + 17));
                    m_position_widget->show();
                    connect(m_position_widget.get(), SIGNAL(updatePosition()), this, SLOT(updateSelectedPosition()));
                    return true;
                }
                return false;
            }



            void RobotInnerWidget::mouseMoveEvent(QMouseEvent* event)
            {
                if (!m_robot)
                {
                    return;
                }

                if (event->button() == Qt::RightButton || !m_selected)
                {
                    return;
                }

                QPoint pos = event->pos();

                QMatrix transform = getTransform();

                bool ok;
                QMatrix inverse_transform = transform.inverted(&ok);
                if (!ok)
                {
                    // throw exception
                }

                QPoint real_point = pos * inverse_transform;
                geometry::Point rp = geometry::Point(real_point.x(), real_point.y());

                m_position_widget->setX(rp.x());
                m_position_widget->setY(rp.y());
                m_robot->setPoint(m_selected_index, geometry::Point(rp.x(), rp.y()));
                m_position_widget->move(QPoint(pos.x() - 90, pos.y() + 17));
                update();

            }

            void RobotInnerWidget::updateNumberOfPoints()
            {
                QPoint p = mapFromGlobal(QCursor::pos());
                bool ok;
                int n = m_robot->polygon().size();
                int n_vertices = QInputDialog::getInt(this, "Change number of points", "Number of points:", n, 3, 2147483647, 1, &ok);
                if (ok)
                {
                    double s;
                    QMatrix transform = getTransform(&s);
                    s /= 250;

                    std::vector< geometry::Point > points;
                    units::Angle step = 360 / n_vertices * units::deg;
                    for (int i = 0; i < n_vertices; i++)
                    {
                        points.emplace_back(units::cos(i * step + 45 * units::deg), units::sin(i * step + 45 * units::deg));
                        points.back() *= units::in();
                    }
                    m_robot->setPolygon(geometry::Polygon(points));
                    update();
                }
            }

            void RobotInnerWidget::updateSelectedPosition()
            {
                QMatrix transform = getTransform();
                m_robot->setPoint(m_selected_index, m_position_widget->position());
                QPoint screen_point = QPoint(m_robot->polygon()[m_selected_index].x()(), m_robot->polygon()[m_selected_index].y()());
                screen_point = screen_point * transform;
                m_position_widget->move(QPoint(screen_point.x() - 45, screen_point.y() + 17));
                update();
            }

            void RobotInnerWidget::paintEvent(QPaintEvent* event)
            {
                QPainter painter;
                painter.begin(this);
                painter.setRenderHint(QPainter::Antialiasing);
                painter.fillRect(event->rect(), Qt::white);

                createBoundingBox();

                double s;
                QMatrix transform = getTransform(&s);
                painter.setTransform(QTransform(transform));
                painter.save();

                s /= 250;

                drawBoundingBox(painter, s);
                drawRobot(painter, s);

                painter.restore();
                painter.end();
            }

            void RobotInnerWidget::resizeEvent(QResizeEvent* event)
            {
                int given_w = event->size().width();
                int given_h = event->size().height();

                int needed_w, needed_h;

                units::Distance min_x, max_x, min_y, max_y;

                geometry::Point center = m_bounding_box.center();
                min_x = center.x() - (center.x() - m_bounding_box.minX()) * 1.2;
                max_x = center.x() + (m_bounding_box.maxX() - center.x()) * 1.2;
                min_y = center.y() - (center.y() - m_bounding_box.minY()) * 1.2;
                max_y = center.y() + (m_bounding_box.maxY() - center.x()) * 1.2;

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

                if (size != event->size())
                {
                    resize(size);
                }
                event->accept();
            }

            QMatrix RobotInnerWidget::getTransform(double* s) const
            {
                QMatrix transform;
                transform.translate(width() / 2.0, height() / 2.0);

                units::Distance min_x, max_x, min_y, max_y;

                geometry::Point center = m_bounding_box.center();
                min_x = center.x() - (center.x() - m_bounding_box.minX()) * 1.2;
                max_x = center.x() + (m_bounding_box.maxX() - center.x()) * 1.2;
                min_y = center.y() - (center.y() - m_bounding_box.minY()) * 1.2;
                max_y = center.y() + (m_bounding_box.maxY() - center.x()) * 1.2;

                geometry::Point scale(max_x - min_x, max_y - min_y);
                if (s)
                {
                    *s = scale.x()();
                }
                transform.scale(width(), -height());
                transform.scale( 1.0 / scale.x()(), 1.0 / scale.y()());

                transform.translate(-center.x()(), -center.y()());
                return transform;
            }

            void RobotInnerWidget::showContextMenu(const QPoint& point)
            {
                QMenu context_menu(tr("Context menu"), this);

                QAction* action;

                action = new QAction("Change number of points", this);
                // action->setIcon(QIcon(":/Icons/add.png"));
                connect(action, SIGNAL(triggered(bool)), this, SLOT(updateNumberOfPoints()));
                context_menu.addAction(action);

                context_menu.addSeparator();

                // todo: delete if right click on point

                context_menu.exec(mapToGlobal(point));
            }

            void RobotInnerWidget::drawRobot(QPainter& painter, const double& scale)
            {
                if (!m_robot)
                {
                    return;
                }

                painter.setPen(QPen(Qt::black, scale));
                painter.setBrush(Qt::yellow);
                QPainterPath path;
                geometry::Polygon poly = m_robot->polygon();
                geometry::Point p = poly.front();
                path.moveTo(p.x()(), p.y()());
                for (int i = 1; i <= poly.size(); i++)
                {
                    p = poly[i % poly.size()];
                    path.lineTo(p.x()(), p.y()());
                }
                painter.drawPath(path);
            }

            void RobotInnerWidget::drawBoundingBox(QPainter& painter, const double& scale)
            {
                painter.setPen(QPen(Qt::gray, scale));
                painter.setBrush(Qt::white);

                painter.drawRect(m_bounding_box.minX()(), m_bounding_box.minY()(), m_bounding_box.width()(), m_bounding_box.height()());
            }

            void RobotInnerWidget::createBoundingBox()
            {
                if(!m_robot)
                {
                    return;
                }

                geometry::Point min(-1 * units::in, -1 * units::in);
                geometry::Point max(1 * units::in, 1 * units::in);

                for (const geometry::Point& point : m_robot->polygon())
                {
                    if (point.x() < min.x())
                    {
                        min.setX(point.x());
                    }

                    if (point.y() < min.y())
                    {
                        min.setY(point.y());
                    }

                    if (point.x() > max.x())
                    {
                        max.setX(point.x());
                    }

                    if (point.y() > max.y())
                    {
                        max.setY(point.y());
                    }
                }

                units::Distance minimum = units::min(min.x(), min.y());
                units::Distance maximum = units::max(max.x(), max.y());

                m_bounding_box = geometry::Rectangle(minimum, maximum, minimum, maximum);
            }
        }
    }
}
