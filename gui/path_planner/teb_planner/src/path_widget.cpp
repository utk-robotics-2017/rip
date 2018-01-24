#include <teb_planner_gui/path_widget.hpp>

#include <QMenu>
#include <QAction>
#include <QPainterPath>
#include <QTransform>

#include "teb_planner/point_obstacle.hpp"
#include "teb_planner/line_obstacle.hpp"
#include "teb_planner/polygon_obstacle.hpp"

#include "teb_planner/point_robot_footprint_model.hpp"
#include "teb_planner/circle_robot_footprint_model.hpp"
#include "teb_planner/polygon_robot_footprint_model.hpp"

namespace rip
{
    namespace gui
    {
        namespace tebplanner
        {

            PathWidget::PathWidget(QWidget* parent)
                : QWidget(parent)
                , m_animate(false)
                , m_timestep(0)
                , m_position_widget(nullptr)
                , m_bounding_box(-1 * units::in, 1 * units::in, -1 * units::in, 1 * units::in)
                , m_start(new navigation::Pose(-0.5 * units::in, 0, 0))
                , m_goal(new navigation::Pose(0.5 * units::in, 0, 0))
                , m_selected_type(SelectedType::kNone)
            {
                m_timer.setInterval(100);
                setContextMenuPolicy(Qt::CustomContextMenu);
            }

            void PathWidget::setAnimate(bool animate)
            {
                m_animate = animate;
                if (m_animate)
                {
                    connect(&m_timer, SIGNAL(timeout()), this, SLOT(update()));
                }
                else
                {
                    disconnect(&m_timer, SIGNAL(timeout()), this, SLOT(update()));
                }

                m_timestep = 0;
                update();
            }

            void PathWidget::setStart(const navigation::Pose& start)
            {
                m_start = std::unique_ptr<navigation::Pose>(new navigation::Pose(start));
                update();
            }

            void PathWidget::setGoal(const navigation::Pose& goal)
            {
                m_goal = std::unique_ptr<navigation::Pose>(new navigation::Pose(goal));
                update();
            }

            void PathWidget::setRobot(std::shared_ptr< navigation::RobotFootprintModel > robot)
            {
                m_robot = robot;
                update();
            }

            void PathWidget::setObstacles(const std::vector<std::shared_ptr<navigation::Obstacle> >& obstacles)
            {
                m_obstacles = obstacles;
                update();
            }

            void PathWidget::updateSelectedPosition()
            {
                QMatrix transform = getTransform();
                QPoint screen_point;

                switch (m_selected_type)
                {
                case SelectedType::kNone:
                {
                    return; // Shouldn't happen...
                }
                case SelectedType::kStartPoint:
                {
                    if(m_bounding_box.clip(m_position_widget->position()) != m_position_widget->position())
                    {
                        m_start->setPosition(m_bounding_box.clip(m_position_widget->position()));
                        m_position_widget->setPosition(m_start->position());
                        screen_point = QPoint(m_start->position().x()(), m_start->position().y()());
                        screen_point = screen_point * transform;
                        m_position_widget->move(QPoint(screen_point.x() - 45, screen_point.y() + 17));
                        update();
                    }
                    else
                    {
                        m_start->setPosition(m_position_widget->position());
                        screen_point = QPoint(m_start->position().x()(), m_start->position().y()());
                        screen_point = screen_point * transform;
                        m_position_widget->move(QPoint(screen_point.x() - 45, screen_point.y() + 17));
                        update();
                    }
                    return;
                }
                case SelectedType::kGoalPoint:
                {
                    if(m_bounding_box.clip(m_position_widget->position()) != m_position_widget->position())
                    {
                        m_goal->setPosition(m_bounding_box.clip(m_position_widget->position()));
                        m_position_widget->setPosition(m_goal->position());
                        screen_point = QPoint(m_goal->position().x()(), m_goal->position().y()());
                        screen_point = screen_point * transform;
                        m_position_widget->move(QPoint(screen_point.x() - 45, screen_point.y() + 17));
                        update();
                    }
                    else
                    {
                        m_goal->setPosition(m_position_widget->position());
                        screen_point = QPoint(m_goal->position().x()(), m_goal->position().y()());
                        screen_point = screen_point * transform;
                        m_position_widget->move(QPoint(screen_point.x() - 45, screen_point.y() + 17));
                        update();
                    }
                    return;
                }
                case SelectedType::kPoint:
                {
                    std::shared_ptr<navigation::PointObstacle> point = std::dynamic_pointer_cast<navigation::PointObstacle>(m_obstacles[m_selected_index]);
                    if(m_bounding_box.clip(m_position_widget->position()) != m_position_widget->position())
                    {
                        point->setPosition(m_bounding_box.clip(m_position_widget->position()));
                        m_position_widget->setPosition(point->position());
                        screen_point = QPoint(point->position().x()(), point->position().y()());
                        screen_point = screen_point * transform;
                        m_position_widget->move(QPoint(screen_point.x() - 45, screen_point.y() + 17));
                        update();
                    }
                    else
                    {
                        point->setPosition(m_position_widget->position());
                        screen_point = QPoint(point->position().x()(), point->position().y()());
                        screen_point = screen_point * transform;
                        m_position_widget->move(QPoint(screen_point.x() - 45, screen_point.y() + 17));
                        update();
                    }
                    return;
                }
                case SelectedType::kLineStart:
                {
                    std::shared_ptr<navigation::LineObstacle> line = std::dynamic_pointer_cast<navigation::LineObstacle>(m_obstacles[m_selected_index]);
                    if(m_bounding_box.clip(line->start()) != m_position_widget->position())
                    {
                        line->setStart(m_bounding_box.clip(m_position_widget->position()));
                        m_position_widget->setPosition(line->start());
                        screen_point = QPoint(line->start().x()(), line->start().y()());
                        screen_point = screen_point * transform;
                        m_position_widget->move(QPoint(screen_point.x() - 45, screen_point.y() + 17));
                        update();
                    }
                    else
                    {
                        line->setStart(m_position_widget->position());
                        screen_point = QPoint(line->start().x()(), line->start().y()());
                        screen_point = screen_point * transform;
                        m_position_widget->move(QPoint(screen_point.x() - 45, screen_point.y() + 17));
                        update();
                    }
                    return;
                }
                case SelectedType::kLineEnd:
                {
                    std::shared_ptr<navigation::LineObstacle> line = std::dynamic_pointer_cast<navigation::LineObstacle>(m_obstacles[m_selected_index]);
                    if(m_bounding_box.clip(line->end()) != m_position_widget->position())
                    {
                        line->setEnd(m_bounding_box.clip(m_position_widget->position()));
                        m_position_widget->setPosition(line->end());
                        screen_point = QPoint(line->end().x()(), line->end().y()());
                        screen_point = screen_point * transform;
                        m_position_widget->move(QPoint(screen_point.x() - 45, screen_point.y() + 17));
                        update();
                    }
                    else
                    {
                        line->setEnd(m_position_widget->position());
                        screen_point = QPoint(line->end().x()(), line->end().y()());
                        screen_point = screen_point * transform;
                        m_position_widget->move(QPoint(screen_point.x() - 45, screen_point.y() + 17));
                        update();
                    }
                    return;
                }
                case SelectedType::kLine:
                {
                    std::shared_ptr<navigation::LineObstacle> line = std::dynamic_pointer_cast<navigation::LineObstacle>(m_obstacles[m_selected_index]);
                    if(m_bounding_box.clip(line->centroid()) != m_position_widget->position())
                    {
                        line->setCentroid(m_bounding_box.clip(m_position_widget->position()));
                        m_position_widget->setPosition(line->centroid());
                        screen_point = QPoint(line->centroid().x()(), line->centroid().y()());
                        screen_point = screen_point * transform;
                        m_position_widget->move(QPoint(screen_point.x() - 45, screen_point.y() + 17));
                        update();
                    }
                    else
                    {
                        line->setCentroid(m_position_widget->position());
                        screen_point = QPoint(line->centroid().x()(), line->centroid().y()());
                        screen_point = screen_point * transform;
                        m_position_widget->move(QPoint(screen_point.x() - 45, screen_point.y() + 17));
                        update();
                    }
                    return;
                }
                }
            }

            void PathWidget::addPointObstacle()
            {
                QPoint p = mapFromGlobal(QCursor::pos());
                QMatrix transform = getTransform();
                bool ok;
                QMatrix inverse_transform = transform.inverted(&ok);
                if(!ok)
                {
                    // todo: throw exception
                }
                p = p * inverse_transform;
                m_obstacles.push_back(std::make_shared<navigation::PointObstacle>(p.x(), p.y()));
                update();
            }

            void PathWidget::addLineObstacle()
            {
                QPoint p = mapFromGlobal(QCursor::pos());
                double s;
                QMatrix transform = getTransform(&s);
                s /= 250;
                bool ok;
                QMatrix inverse_transform = transform.inverted(&ok);
                if(!ok)
                {
                    // todo: throw exception
                }
                p = p * inverse_transform;
                m_obstacles.push_back(std::make_shared<navigation::LineObstacle>(p.x(), p.y(), p.x(), p.y() + 15 * s));
                //createBoundingBox();
                update();
            }

            void PathWidget::addPolygonObstacle()
            {
                QPoint p = mapFromGlobal(QCursor::pos());
            }

            void PathWidget::paintEvent(QPaintEvent* event)
            {
                QPainter painter;
                painter.begin(this);
                painter.setRenderHint(QPainter::Antialiasing);
                painter.fillRect(event->rect(), Qt::white);

                double s;
                QMatrix transform = getTransform(&s);
                painter.setTransform(QTransform(transform));
                painter.save();

                s /= 250;

                drawBoundingBox(painter, s);
                drawObstacles(painter, s);
                drawTrajectory(painter, s);
                drawStartAndGoal(painter, s);
                drawWaypoints(painter, s);
                drawRobot(painter, s);

                painter.restore();
                painter.end();
            }

            void PathWidget::resizeEvent(QResizeEvent* event)
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

            void PathWidget::mousePressEvent(QMouseEvent* event)
            {
                if(event->button() == Qt::RightButton)
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
                if(m_bounding_box.clip(real_point) != real_point)
                {
                    return;
                }

                s /= 250;

                QPoint screen_point;

                if(m_start)
                {
                    if(checkPointClick(m_start->position(), transform, pos, SelectedType::kStartPoint))
                    {
                        return;
                    }

                    geometry::Point tip_offset = m_start->orientationUnitVector() * 10 * s;
                    geometry::Point tip_location = m_start->position() + tip_offset;

                    if(checkPointClick(tip_location, transform, pos, SelectedType::kStartDirection))
                    {
                        return;
                    }
                }

                if(m_goal)
                {
                    if(checkPointClick(m_goal->position(), transform, pos, SelectedType::kGoalPoint))
                    {
                        return;
                    }

                    geometry::Point tip_offset = m_goal->orientationUnitVector() * 10 * s;
                    geometry::Point tip_location = m_goal->position() + tip_offset;

                    if(checkPointClick(tip_location, transform, pos, SelectedType::kGoalDirection))
                    {
                        return;
                    }
                }

                for(int idx = 0; idx < m_obstacles.size(); idx++)
                {
                    std::shared_ptr<navigation::Obstacle> obstacle = m_obstacles[idx];

                    std::shared_ptr<navigation::PointObstacle> point = std::dynamic_pointer_cast<navigation::PointObstacle>(obstacle);
                    if(point)
                    {
                        if(checkPointClick(point->centroid(), transform, pos, SelectedType::kPoint, idx))
                        {
                            return;
                        }
                        continue;
                    }

                    std::shared_ptr<navigation::LineObstacle> line = std::dynamic_pointer_cast<navigation::LineObstacle>(obstacle);
                    if(line)
                    {
                        if(checkPointClick(line->centroid(), transform, pos, SelectedType::kLine, idx))
                        {
                            return;
                        }

                        if(checkPointClick(line->start(), transform, pos, SelectedType::kLineStart, idx))
                        {
                            return;
                        }

                        if(checkPointClick(line->end(), transform, pos, SelectedType::kLineEnd, idx))
                        {
                            return;
                        }
                    }
                }

                /*
                for(int idx = 0; idx < m_waypoints.size(); idx++)
                {
                    const geometry::Point& point = m_waypoints[idx];
                    screen_point = QPoint(point.x()(), point.y()());
                    screen_point *= transform;
                    if((screen_point - pos).manhattanLength() < pixel_threshold)
                    {
                        m_selected_type = SelectedType::kWaypoint;
                        m_selected_index = idx;


                        // todo: display labels and textboxes
                        return;
                    }
                }

                for(int idx = 0; idx < m_obstacles.size(); idx++)
                {
                    std::shared_ptr<navigation::Obstacle> obstacle = m_obstacles[idx];

                    std::shared_ptr<navigation::PointObstacle> point = m_obstacles[idx];
                    if(point)
                    {
                        screen_point = QPoint(point->centroid().x(), point->centroid().y());
                        screen_point *= transform;
                        if((screen_point - pos).manhattanLength() < pixel_threshold)
                        {
                            m_selected_type = SelectedType::kObstacle;
                            m_selected_index = idx;

                            // todo: display labels and textbox
                            return;
                        }

                        continue;
                    }

                    std::shared_ptr<navigation::LineObstacle> line = m_obstacles[idx];
                    if(line)
                    {
                        screen_point = QPoint(line->start().x()(), line->start().y()());
                        screen_point *= transform;
                        if((screen_point - pos).manhattanLength() < pixel_threshold)
                        {
                            m_selected_type = SelectedType::kLineStart;
                            m_selected_index = idx;

                            // todo: display labels and textbox
                            return;
                        }

                        screen_point = QPoint(line->end().x()(), line->end().y()());
                        screen_point *= transform;
                        if((screen_point - pos).manhattanLength() < pixel_threshold)
                        {
                            m_selected_type = SelectedType::kLineEnd;
                            m_selected_index = idx;

                            // todo: display labels and textbox
                            return;
                        }

                        continue;
                    }
                }
                */

                if(m_position_widget)
                {
                    m_position_widget->hide();
                    disconnect(m_position_widget.get(), SIGNAL(updatePosition()), this, SLOT(updateSelectedPosition()));
                    m_position_widget.reset();
                    m_selected_type = SelectedType::kNone;
                }
            }

            void PathWidget::mouseMoveEvent(QMouseEvent* event)
            {
                if(event->button() == Qt::RightButton || m_selected_type == SelectedType::kNone)
                {
                    return;
                }

                QPoint pos = event->pos();

                QMatrix transform = getTransform();

                bool ok;
                QMatrix inverse_transform = transform.inverted(&ok);
                if(!ok)
                {
                    // throw exception
                }

                QPoint real_point;
                geometry::Point rp;

                real_point = pos * inverse_transform;
                geometry::Point clipped_point = m_bounding_box.clip(geometry::Point(real_point.x(), real_point.y()));
                pos = QPoint(clipped_point.x()(), clipped_point.y()()) * transform;

                switch (m_selected_type)
                {
                case SelectedType::kStartPoint:
                {
                    real_point = pos * inverse_transform;
                    rp = geometry::Point(real_point.x(), real_point.y());
                    m_position_widget->setX(rp.x());
                    m_position_widget->setY(rp.y());
                    m_start->setX(rp.x());
                    m_start->setY(rp.y());
                    m_position_widget->move(QPoint(pos.x() - 90, pos.y() + 17));
                    update();
                    return;
                }
                case SelectedType::kGoalPoint:
                {
                    real_point = pos * inverse_transform;
                    rp = geometry::Point(real_point.x(), real_point.y());
                    m_position_widget->setX(rp.x());
                    m_position_widget->setY(rp.y());
                    m_goal->setX(rp.x());
                    m_goal->setY(rp.y());
                    m_position_widget->move(QPoint(pos.x() - 90, pos.y() + 17));
                    update();
                    return;
                }
                case SelectedType::kPoint:
                {
                    real_point = pos * inverse_transform;
                    rp = geometry::Point(real_point.x(), real_point.y());
                    m_position_widget->setX(rp.x());
                    m_position_widget->setY(rp.y());
                    std::shared_ptr<navigation::PointObstacle> p_obstacle = std::dynamic_pointer_cast<navigation::PointObstacle>(m_obstacles[m_selected_index]);
                    p_obstacle->setX(rp.x());
                    p_obstacle->setY(rp.y());
                    m_position_widget->move(QPoint(pos.x() - 90, pos.y() + 17));
                    update();
                    return;
                }
                case SelectedType::kLineStart:
                {
                    real_point = pos * inverse_transform;
                    rp = geometry::Point(real_point.x(), real_point.y());
                    m_position_widget->setX(rp.x());
                    m_position_widget->setY(rp.y());
                    std::shared_ptr<navigation::LineObstacle> l_obstacle = std::dynamic_pointer_cast<navigation::LineObstacle>(m_obstacles[m_selected_index]);
                    l_obstacle->setStart(geometry::Point(rp.x(), rp.y()));
                    m_position_widget->move(QPoint(pos.x() - 90, pos.y() + 17));
                    update();
                    return;
                }
                case SelectedType::kLineEnd:
                {
                    real_point = pos * inverse_transform;
                    rp = geometry::Point(real_point.x(), real_point.y());
                    m_position_widget->setX(rp.x());
                    m_position_widget->setY(rp.y());
                    std::shared_ptr<navigation::LineObstacle> l_obstacle = std::dynamic_pointer_cast<navigation::LineObstacle>(m_obstacles[m_selected_index]);
                    l_obstacle->setEnd(geometry::Point(rp.x(), rp.y()));
                    m_position_widget->move(QPoint(pos.x() - 90, pos.y() + 17));
                    update();
                    return;
                }
                case SelectedType::kLine:
                {
                    real_point = pos * inverse_transform;
                    rp = geometry::Point(real_point.x(), real_point.y());
                    m_position_widget->setX(rp.x());
                    m_position_widget->setY(rp.y());
                    std::shared_ptr<navigation::LineObstacle> l_obstacle = std::dynamic_pointer_cast<navigation::LineObstacle>(m_obstacles[m_selected_index]);
                    l_obstacle->setCentroid(geometry::Point(rp.x(), rp.y()));
                    m_position_widget->move(QPoint(pos.x() - 90, pos.y() + 17));
                    update();
                    return;
                }
                }
            }

            void PathWidget::mouseReleaseEvent(QMouseEvent* event)
            {
                /*
                if(m_position_widget)
                {
                    m_position_widget->hide();
                    m_position_widget.reset();
                }
                m_selected_type = SelectedType::kNone;
                */
            }

            QMatrix PathWidget::getTransform(double* s) const
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
                if(s)
                {
                    *s = scale.x()();
                }
                transform.scale(width(), -height());
                transform.scale( 1.0 / scale.x()(), 1.0 / scale.y()());

                transform.translate(-center.x()(), -center.y()());
                return transform;
            }

            void PathWidget::drawBoundingBox(QPainter& painter, double scale)
            {
                painter.setPen(QPen(Qt::gray, scale));
                painter.setBrush(Qt::white);

                painter.drawRect(m_bounding_box.minX()(), m_bounding_box.minY()(), m_bounding_box.width()(), m_bounding_box.height()());
            }

            void PathWidget::drawStartAndGoal(QPainter& painter, double scale)
            {
                drawPose(*m_start, Qt::green, painter, scale);
                drawPose(*m_goal, Qt::red, painter, scale);
            }

            void PathWidget::drawPose(const navigation::Pose& pose, Qt::GlobalColor color, QPainter& painter, double scale)
            {
                int diameter = scale * 5;

                // Draw point
                painter.setPen(QPen(Qt::black, 1));
                painter.setBrush(color);
                painter.drawEllipse(pose.position().x()() - diameter / 2.0, pose.position().y()() - diameter / 2.0, diameter, diameter);

                geometry::Point tip_offset = pose.orientationUnitVector() * 10 * scale;
                geometry::Point tip_location = pose.position() + tip_offset;

                // Draw arrow shaft
                painter.setPen(QPen(color, scale));
                painter.drawLine(pose.position().x()(), pose.position().y()(), tip_location.x()(), tip_location.y()());

                // Draw arrow head
                geometry::Point head_base = tip_location - tip_offset.normalize() * 2 * scale;
                geometry::Point perp(-tip_offset.y(), tip_offset.x());
                perp = perp.normalize();
                geometry::Point tip_left = head_base + perp * 2 * scale;
                geometry::Point tip_right = head_base - perp * 2 * scale;
                QPointF triangle_points[] = {QPointF(tip_location.x()(), tip_location.y()()), QPointF(tip_left.x()(), tip_left.y()()), QPointF(tip_right.x()(), tip_right.y()())};
                painter.drawPolygon(triangle_points, 3);
            }

            void PathWidget::showContextMenu(const QPoint& point)
            {
                QMenu context_menu(tr("Context menu"), this);

                QAction* action;

                action = new QAction("Add Point Obstacle", this);
                action->setIcon(QIcon(":/Icons/add.png"));
                connect(action, SIGNAL(triggered(bool)), this, SLOT(addPointObstacle()));
                context_menu.addAction(action);

                action = new QAction("Add Line Obstacle", this);
                action->setIcon(QIcon(":/Icons/add.png"));
                connect(action, SIGNAL(triggered(bool)), this, SLOT(addLineObstacle()));
                context_menu.addAction(action);

                action = new QAction("Add Polygon Obstacle", this);
                action->setIcon(QIcon(":/Icons/add.png"));
                connect(action, SIGNAL(triggered(bool)), this, SLOT(addPolygonObstacle()));
                context_menu.addAction(action);

                context_menu.addSeparator();

                // todo: delete if right click on point

                context_menu.exec(mapToGlobal(point));
            }

            bool PathWidget::checkPointClick(const geometry::Point& point, const QMatrix& transform, const QPoint& mouse_pos, PathWidget::SelectedType select, int index, int polygon_index, unsigned int pixel_threshold)
            {
                QPoint screen_point(point.x()(), point.y()());
                screen_point = screen_point * transform;
                if((screen_point - mouse_pos).manhattanLength() < pixel_threshold)
                {
                    m_selected_type = select;
                    m_selected_index = index;
                    m_selected_polygon_index = polygon_index;

                    m_position_widget = std::unique_ptr<PositionWidget>(new PositionWidget(point, this));
                    m_position_widget->move(QPoint(screen_point.x() - 90, screen_point.y() + 17));
                    m_position_widget->show();
                    connect(m_position_widget.get(), SIGNAL(updatePosition()), this, SLOT(updateSelectedPosition()));
                    return true;
                }
                return false;
            }

            void PathWidget::drawObstacles(QPainter& painter, double scale)
            {
                if (m_obstacles.empty())
                {
                    return;
                }

                painter.setPen(QPen(Qt::black, scale));
                painter.setBrush(Qt::black);

                int diameter = scale * 5;

                for (std::shared_ptr< navigation::Obstacle > obstacle : m_obstacles)
                {
                    std::shared_ptr<navigation::PointObstacle> point = std::dynamic_pointer_cast<navigation::PointObstacle>(obstacle);
                    if (point)
                    {
                        painter.drawEllipse(point->centroid().x()() - diameter / 2.0, point->centroid().y()() - diameter / 2.0, diameter, diameter);

                        continue;
                    }

                    std::shared_ptr< navigation::LineObstacle > line = std::dynamic_pointer_cast< navigation::LineObstacle >(obstacle);
                    if (line)
                    {
                        painter.drawLine(line->start().x()(), line->start().y()(), line->end().x()(), line->end().y()());
                        continue;
                    }

                    std::shared_ptr< navigation::PolygonObstacle > polygon = std::dynamic_pointer_cast< navigation::PolygonObstacle >(obstacle);
                    if (polygon)
                    {
                        QPainterPath path;
                        geometry::Polygon poly = polygon->polygon();
                        geometry::Point p = poly.front();
                        path.moveTo(p.x()(), p.y()());
                        for(int i = 1; i <= poly.size(); i++)
                        {
                            p = poly[i % poly.size()];
                            path.lineTo(p.x()(), p.y()());
                        }
                        painter.drawPath(path);
                        continue;
                    }

                    // throw error
                }
            }

            void PathWidget::drawTrajectory(QPainter& painter, double scale)
            {
                if (m_trajectory.empty())
                {
                    return;
                }

                painter.setPen(QPen(Qt::red, scale));
                painter.setBrush(Qt::red);

                bool first = true;
                geometry::Point p0;

                for (const navigation::TrajectoryPoint& point : m_trajectory)
                {
                    if(first)
                    {
                        p0 = point.position();
                        continue;
                    }

                    geometry::Point p1 = point.position();

                    painter.drawLine(p0.x()(), p0.y()(), p1.x()(), p1.y()());
                    p1 = p0;
                }
            }

            void PathWidget::drawWaypoints(QPainter& painter, double scale)
            {
                if (m_waypoints.empty())
                {
                    return;
                }

                painter.setPen(QPen(Qt::black, scale));
                painter.setBrush(Qt::yellow);
                int diameter = scale * 5;
                for (const geometry::Point& waypoint : m_waypoints)
                {
                    painter.drawEllipse(waypoint.x()() - diameter / 2.0, waypoint.y()() - diameter / 2.0, diameter, diameter);
                }
            }

            void PathWidget::drawRobot(QPainter& painter, double scale)
            {
                if (m_animate && m_robot && m_trajectory.size())
                {
                    if (m_timestep == m_trajectory.size())
                    {
                        m_timestep = 0;
                    }

                    geometry::Point position = m_trajectory[m_timestep].position();
                    units::Angle rotation = m_trajectory[m_timestep].theta();

                    m_timestep ++;

                    std::shared_ptr< navigation::PointRobotFootprintModel > point = std::dynamic_pointer_cast< navigation::PointRobotFootprintModel >(m_robot);
                    if(point)
                    {
                        int diameter = scale * 5;
                        painter.drawEllipse(position.x()() - diameter / 2.0, position.y()() - diameter / 2.0, diameter, diameter);
                    }

                    std::shared_ptr< navigation::CircleRobotFootprintModel > circle = std::dynamic_pointer_cast< navigation::CircleRobotFootprintModel >(m_robot);
                    if(circle)
                    {
                        int diameter = circle->inscribedRadius()() * 2;
                        painter.drawEllipse(position.x()() - diameter / 2.0, position.y()() - diameter / 2.0, diameter, diameter);
                    }

                    std::shared_ptr< navigation::PolygonRobotFootprintModel > polygon = std::dynamic_pointer_cast< navigation::PolygonRobotFootprintModel >(m_robot);
                    if(polygon)
                    {
                        geometry::Polygon poly = polygon->polygon(position, rotation);

                        QPainterPath path;

                        geometry::Point p = poly.front();
                        path.moveTo(p.x()(), p.y()());
                        for(int i = 1; i < poly.size(); i++)
                        {
                            p = poly[i];
                            path.lineTo(p.x()(), p.y()());
                        }
                        painter.drawPath(path);
                    }
                }

            }

            void PathWidget::createBoundingBox()
            {
                if(m_obstacles.empty())
                {
                    m_bounding_box = geometry::Rectangle(-1 * units::in, 1 * units::in, -1 * units::in, 1 * units::in);
                    return;
                }

                geometry::Point min(std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
                geometry::Point max(std::numeric_limits<double>::min(), std::numeric_limits<double>::min());

                for(std::shared_ptr< navigation::Obstacle > obstacle : m_obstacles)
                {
                    std::shared_ptr< navigation::PointObstacle > point = std::dynamic_pointer_cast< navigation::PointObstacle >(obstacle);
                    if (point)
                    {
                        if(point->centroid().x() < min.x())
                        {
                            min.setX(point->centroid().x());
                        }

                        if(point->centroid().y() < min.y())
                        {
                            min.setY(point->centroid().y());
                        }

                        if(point->centroid().x() > max.x())
                        {
                            max.setX(point->centroid().x());
                        }

                        if(point->centroid().y() > max.y())
                        {
                            max.setY(point->centroid().y());
                        }

                        continue;
                    }

                    std::shared_ptr< navigation::LineObstacle > line = std::dynamic_pointer_cast< navigation::LineObstacle >(obstacle);
                    if (line)
                    {
                        if(line->start().x() < min.x())
                        {
                            min.setX(line->start().x());
                        }

                        if(line->start().y() < min.y())
                        {
                            min.setY(line->start().y());
                        }

                        if(line->start().x() > max.x())
                        {
                            max.setX(line->start().x());
                        }

                        if(line->start().y() > max.y())
                        {
                            max.setY(line->start().y());
                        }

                        if(line->end().x() < min.x())
                        {
                            min.setX(line->end().x());
                        }

                        if(line->end().y() < min.y())
                        {
                            min.setY(line->end().y());
                        }

                        if(line->end().x() > max.x())
                        {
                            max.setX(line->end().x());
                        }

                        if(line->end().y() > max.y())
                        {
                            max.setY(line->end().y());
                        }

                        continue;
                    }

                    std::shared_ptr< navigation::PolygonObstacle> polygon = std::dynamic_pointer_cast< navigation::PolygonObstacle >(obstacle);
                    if (polygon)
                    {
                        for(const geometry::Point& point : polygon->polygon())
                        {
                            if(point.x() < min.x())
                            {
                                min.setX(point.x());
                            }

                            if(point.y() < min.y())
                            {
                                min.setY(point.y());
                            }

                            if(point.x() > max.x())
                            {
                                max.setX(point.x());
                            }

                            if(point.y() > max.y())
                            {
                                max.setY(point.y());
                            }
                        }
                    }
                }

                m_bounding_box = geometry::Rectangle(min.x(), max.y(), max.x(), min.y());
            }

        }
    }
}
