#include <teb_planner_gui/path_inner_widget.hpp>

#include <QMenu>
#include <QAction>
#include <QPainterPath>
#include <QTransform>
#include <QInputDialog>

namespace rip
{
    namespace gui
    {
        namespace tebplanner
        {

            PathInnerWidget::PathInnerWidget(QWidget* parent)
                : QWidget(parent)
                , m_animate(false)
                , m_timestep(0)
                , m_position_widget(nullptr)
                , m_bounding_box(-1 * units::in, 1 * units::in, -1 * units::in, 1 * units::in)
                , m_obstacles(nullptr)
                , m_start(new navigation::tebplanner::Pose(-1 * units::m, 0, 0))
                , m_goal(new navigation::tebplanner::Pose(1 * units::m, 0, 0))
                , m_selected_type(SelectedType::kNone)
                , m_compute_thread(ComputeThread::getInstance())
            {
                m_compute_thread->setStart(m_start);
                m_compute_thread->setGoal(m_goal);

                m_timer.setInterval(100);
                setContextMenuPolicy(Qt::CustomContextMenu);

                connect(m_compute_thread.get(), SIGNAL(trajectoryUpdated()), this, SLOT(trajectoryUpdated()));
            }

            void PathInnerWidget::setAnimate(bool animate)
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

            void PathInnerWidget::setRobot(std::shared_ptr< navigation::tebplanner::BaseRobotFootprintModel > robot)
            {
                m_robot = robot;
                update();
            }

            void PathInnerWidget::setObstacles(std::shared_ptr< std::vector< std::shared_ptr< navigation::tebplanner::Obstacle > > > obstacles)
            {
                m_obstacles = obstacles;
                update();
            }

            void PathInnerWidget::updateSelectedPosition()
            {
                double scale;
                QMatrix transform = getTransform(&scale);
                QPoint screen_point;

                switch (m_selected_type)
                {
                case SelectedType::kNone:
                {
                    return; // Shouldn't happen...
                }
                case SelectedType::kStartPoint:
                {
                    m_start->setPosition(m_position_widget->position());
                    screen_point = QPoint(m_start->position().x()(), m_start->position().y()());
                    break;
                }
                case SelectedType::kStartDirection:
                {
                    m_start->setTheta(m_angle_widget->theta());
                    geometry::Point orientation = m_start->orientationUnitVector() * 5 * scale;
                    screen_point = QPoint(m_start->position().x()(), m_start->position().y()());

                    screen_point = screen_point * transform;
                    int x = std::max(std::min(screen_point.x() - 45, width()-250), 100);
                    int y = std::max(std::min(screen_point.y() + 17, height() - 115), 60);
                    m_angle_widget->move(QPoint(x,y));
                    update();
                    return;
                }
                case SelectedType::kGoalPoint:
                {
                    m_goal->setPosition(m_position_widget->position());
                    screen_point = QPoint(m_goal->position().x()(), m_goal->position().y()());
                    break;
                }
                case SelectedType::kGoalDirection:
                {
                    m_goal->setTheta(m_angle_widget->theta());
                    geometry::Point orientation = m_goal->orientationUnitVector() * 5 * scale;
                    screen_point = QPoint(m_goal->position().x()(), m_goal->position().y()());

                    screen_point = screen_point * transform;
                    int x = std::max(std::min(screen_point.x() - 45, width()-250), 100);
                    int y = std::max(std::min(screen_point.y() + 17, height() - 115), 60);
                    m_angle_widget->move(QPoint(x,y));
                    update();
                    return;
                }
                case SelectedType::kPoint:
                {
                    std::shared_ptr<navigation::tebplanner::PointObstacle> point = std::dynamic_pointer_cast<navigation::tebplanner::PointObstacle>((*m_obstacles)[m_selected_index]);
                    point->setPosition(m_position_widget->position());
                    screen_point = QPoint(point->point().x()(), point->point().y()());
                    break;
                }
                case SelectedType::kLineStart:
                {
                    std::shared_ptr<navigation::tebplanner::LineObstacle> line = std::dynamic_pointer_cast<navigation::tebplanner::LineObstacle>((*m_obstacles)[m_selected_index]);
                    line->setStart(m_position_widget->position());
                    screen_point = QPoint(line->startPoint().x()(), line->startPoint().y()());
                    break;
                }
                case SelectedType::kLineEnd:
                {
                    std::shared_ptr<navigation::tebplanner::LineObstacle> line = std::dynamic_pointer_cast<navigation::tebplanner::LineObstacle>((*m_obstacles)[m_selected_index]);
                    line->setEnd(m_position_widget->position());
                    screen_point = QPoint(line->endPoint().x()(), line->endPoint().y()());
                    break;
                }
                case SelectedType::kLine:
                {
                    std::shared_ptr<navigation::tebplanner::LineObstacle> line = std::dynamic_pointer_cast<navigation::tebplanner::LineObstacle>((*m_obstacles)[m_selected_index]);
                    line->setCentroid(m_position_widget->position());
                    screen_point = QPoint(line->getCentroidPoint().x()(), line->getCentroidPoint().y()());
                    break;
                }
                case SelectedType::kPolygonPoint:
                {
                    std::shared_ptr<navigation::tebplanner::PolygonObstacle> polygon = std::dynamic_pointer_cast<navigation::tebplanner::PolygonObstacle>((*m_obstacles)[m_selected_index]);
                    polygon->setPoint(m_selected_polygon_index, m_position_widget->position());
                    screen_point = QPoint(polygon->polygon()[m_selected_polygon_index].x()(), polygon->polygon()[m_selected_polygon_index].y()());
                    break;
                }
                case SelectedType::kPolygon:
                {
                    std::shared_ptr<navigation::tebplanner::PolygonObstacle> polygon = std::dynamic_pointer_cast<navigation::tebplanner::PolygonObstacle>((*m_obstacles)[m_selected_index]);
                    polygon->setCentroid(m_position_widget->position());
                    screen_point = QPoint(polygon->getCentroidPoint().x()(), polygon->getCentroidPoint().y()());
                    break;
                }
                }
                screen_point = screen_point * transform;
                int x = std::max(std::min(screen_point.x() - 45, width()-250), 100);
                int y = std::max(std::min(screen_point.y() + 17, height() - 115), 60);
                m_position_widget->move(QPoint(x,y));
                update();
            }

            void PathInnerWidget::addPointObstacle()
            {
                QPoint p = mapFromGlobal(QCursor::pos());
                QMatrix transform = getTransform();
                bool ok;
                QMatrix inverse_transform = transform.inverted(&ok);
                if (!ok)
                {
                    // todo: throw exception
                }
                p = p * inverse_transform;
                geometry::Point rp(p.x(), p.y());
                m_obstacles->emplace_back(std::make_shared<navigation::tebplanner::PointObstacle>(rp));
                update();
            }

            void PathInnerWidget::addLineObstacle()
            {
                QPoint p = mapFromGlobal(QCursor::pos());
                double s;
                QMatrix transform = getTransform(&s);
                s /= 250;
                bool ok;
                QMatrix inverse_transform = transform.inverted(&ok);
                if (!ok)
                {
                    // todo: throw exception
                }
                p = p * inverse_transform;
                geometry::Point start(p.x(), p.y());
                geometry::Point end(p.x(), p.y() + 15 * s);
                m_obstacles->emplace_back(std::make_shared<navigation::tebplanner::LineObstacle>(start, end));
                update();
            }

            void PathInnerWidget::addPolygonObstacle()
            {
                QPoint p = mapFromGlobal(QCursor::pos());
                bool ok;
                int n_vertices = QInputDialog::getInt(this, "Add Polygon Obstacle", "Number of vertices:", 3, 3, 2147483647, 1, &ok);
                if (ok)
                {
                    double s;
                    QMatrix transform = getTransform(&s);
                    s /= 250;

                    std::vector< geometry::Point > points;
                    units::Angle step = 360 / n_vertices * units::deg;
                    for (int i = 0; i < n_vertices; i++)
                    {
                        points.emplace_back(units::cos(i * step), units::sin(i * step));
                        points.back() *= 15 * s;
                    }
                    m_obstacles->emplace_back(std::make_shared<navigation::tebplanner::PolygonObstacle>(points));
                    update();
                }
            }

            void PathInnerWidget::trajectoryUpdated()
            {
                m_trajectory = m_compute_thread->trajectory();
                update();
            }

            void PathInnerWidget::paintEvent(QPaintEvent* event)
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
                drawObstacles(painter, s);
                drawTrajectory(painter, s);
                drawStartAndGoal(painter, s);
                drawWaypoints(painter, s);
                drawRobot(painter, s);

                painter.restore();
                painter.end();
            }

            void PathInnerWidget::resizeEvent(QResizeEvent* event)
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

            void PathInnerWidget::mousePressEvent(QMouseEvent* event)
            {
                if (event->button() == Qt::RightButton)
                {
                    if(m_obstacles)
                    {
                        showContextMenu(event->pos());
                    }
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

                if (m_start)
                {
                    if (checkPointClick(m_start->position(), transform, pos, SelectedType::kStartPoint))
                    {
                        return;
                    }

                    geometry::Point tip_offset = m_start->orientationUnitVector() * 10 * s;
                    geometry::Point tip_location = m_start->position() + tip_offset;

                    QPoint screen_point(tip_location.x()(), tip_location.y()());
                    screen_point = screen_point * transform;
                    if ((screen_point - pos).manhattanLength() < 15)
                    {
                        pos = QPoint(m_start->position().x()(), m_start->position().y()());
                        pos = pos * transform;

                        int x = std::max(std::min(pos.x() - 90, width() - 250), 75);
                        int y = std::max(std::min(pos.y() + 17, height() - 115), 75);

                        m_selected_type = SelectedType::kStartDirection;

                        m_angle_widget = std::unique_ptr<AngleWidget>(new AngleWidget(m_start->theta(), this));
                        m_angle_widget->move(QPoint(x, y));
                        m_angle_widget->show();
                        connect(m_angle_widget.get(), SIGNAL(updateAngle()), this, SLOT(updateSelectedPosition()));
                        return;
                    }
                }

                if (m_goal)
                {
                    if (checkPointClick(m_goal->position(), transform, pos, SelectedType::kGoalPoint))
                    {
                        return;
                    }

                    geometry::Point tip_offset = m_goal->orientationUnitVector() * 10 * s;
                    geometry::Point tip_location = m_goal->position() + tip_offset;

                    QPoint screen_point(tip_location.x()(), tip_location.y()());
                    screen_point = screen_point * transform;
                    if ((screen_point - pos).manhattanLength() < 15)
                    {
                        pos = QPoint(m_goal->position().x()(), m_goal->position().y()());
                        pos = pos * transform;

                        int x = std::max(std::min(pos.x() - 90, width() - 250), 75);
                        int y = std::max(std::min(pos.y() + 17, height() - 115), 75);

                        m_selected_type = SelectedType::kGoalDirection;

                        m_angle_widget = std::unique_ptr<AngleWidget>(new AngleWidget(m_goal->theta(), this));
                        m_angle_widget->move(QPoint(x, y));
                        m_angle_widget->show();
                        connect(m_angle_widget.get(), SIGNAL(updateAngle()), this, SLOT(updateSelectedPosition()));
                        return;
                    }
                }

                if(m_obstacles && m_obstacles->size())
                {
                    int x = std::max(std::min(pos.x() - 90, width() - 250), 75);
                    int y = std::max(std::min(pos.y() + 17, height() - 115), 75);
                    for (int idx = 0; idx < m_obstacles->size(); idx++)
                    {
                        std::shared_ptr<navigation::tebplanner::Obstacle> obstacle = (*m_obstacles)[idx];

                        std::shared_ptr<navigation::tebplanner::PointObstacle> point = std::dynamic_pointer_cast<navigation::tebplanner::PointObstacle>(obstacle);
                        if (point)
                        {
                            if (checkPointClick(point->getCentroidPoint(), transform, pos, SelectedType::kPoint, idx))
                            {
                                return;
                            }
                            continue;
                        }

                        std::shared_ptr<navigation::tebplanner::LineObstacle> line = std::dynamic_pointer_cast<navigation::tebplanner::LineObstacle>(obstacle);
                        if (line)
                        {
                            if (checkPointClick(line->startPoint(), transform, pos, SelectedType::kLineStart, idx))
                            {
                                return;
                            }

                            if (checkPointClick(line->endPoint(), transform, pos, SelectedType::kLineEnd, idx))
                            {
                                return;
                            }

                            if (checkPointClick(line->getCentroidPoint(), transform, pos, SelectedType::kLine, idx))
                            {
                                return;
                            }
                        }

                        std::shared_ptr<navigation::tebplanner::PolygonObstacle> polygon = std::dynamic_pointer_cast<navigation::tebplanner::PolygonObstacle>(obstacle);
                        if (polygon)
                        {
                            geometry::Polygon poly = polygon->polygon();
                            for (int i = 0; i < poly.size(); i++)
                            {
                                if (checkPointClick(poly[i], transform, pos, SelectedType::kPolygonPoint, idx, i))
                                {
                                    return;
                                }
                            }

                            if (checkPointClick(polygon->getCentroidPoint(), transform, pos, SelectedType::kPolygon, idx))
                            {
                                return;
                            }


                            if (polygon->polygon().inside(real_point))
                            {
                                m_selected_type = SelectedType::kPolygon;
                                m_selected_index = idx;

                                m_position_widget = std::unique_ptr<PositionWidget>(new PositionWidget(polygon->getCentroidPoint(), this));
                                m_position_widget->move(QPoint(x, y));
                                m_position_widget->show();
                                connect(m_position_widget.get(), SIGNAL(updatePosition()), this, SLOT(updateSelectedPosition()));
                                return;
                            }
                        }
                    }
                }

                if (m_position_widget)
                {
                    m_position_widget->hide();
                    disconnect(m_position_widget.get(), SIGNAL(updatePosition()), this, SLOT(updateSelectedPosition()));
                    m_position_widget.reset();
                    m_selected_type = SelectedType::kNone;
                }

                if(m_angle_widget)
                {
                    m_angle_widget->hide();
                    disconnect(m_angle_widget.get(), SIGNAL(updateAngle()), this, SLOT(updateSelectedPosition()));
                    m_angle_widget.reset();
                    m_selected_type = SelectedType::kNone;
                }
            }

            void PathInnerWidget::mouseMoveEvent(QMouseEvent* event)
            {
                if (event->button() == Qt::RightButton || m_selected_type == SelectedType::kNone)
                {
                    return;
                }

                QPoint pos = event->pos();
                int x = std::max(std::min(pos.x() - 90, width() - 250), 75);
                int y = std::max(std::min(pos.y() + 17, height() - 115), 75);

                QMatrix transform = getTransform();

                bool ok;
                QMatrix inverse_transform = transform.inverted(&ok);
                if (!ok)
                {
                    // throw exception
                }

                QPoint real_point = pos * inverse_transform;
                geometry::Point rp = geometry::Point(real_point.x(), real_point.y());

                switch (m_selected_type)
                {
                case SelectedType::kStartPoint:
                {
                    m_start->setPosition(rp);
                    break;
                }
                case SelectedType::kStartDirection:
                {
                    geometry::Point diff = rp - m_start->position();
                    units::Angle theta = geometry::atan(diff);
                    m_start->setTheta(theta);
                    m_angle_widget->setTheta(theta);
                    update();
                    return;
                }
                case SelectedType::kGoalPoint:
                {
                    m_goal->setPosition(rp);
                    break;

                }
                case SelectedType::kGoalDirection:
                {
                    geometry::Point diff = rp - m_goal->position();
                    units::Angle theta = geometry::atan(diff);
                    m_goal->setTheta(theta);
                    m_angle_widget->setTheta(theta);
                    update();
                    return;
                }
                case SelectedType::kPoint:
                {
                    std::shared_ptr<navigation::tebplanner::PointObstacle> p_obstacle = std::dynamic_pointer_cast<navigation::tebplanner::PointObstacle>((*m_obstacles)[m_selected_index]);
                    p_obstacle->setPosition(rp);
                    break;
                }
                case SelectedType::kLineStart:
                {
                    std::shared_ptr<navigation::tebplanner::LineObstacle> l_obstacle = std::dynamic_pointer_cast<navigation::tebplanner::LineObstacle>((*m_obstacles)[m_selected_index]);
                    l_obstacle->setStart(rp);
                    break;
                }
                case SelectedType::kLineEnd:
                {
                    std::shared_ptr<navigation::tebplanner::LineObstacle> l_obstacle = std::dynamic_pointer_cast<navigation::tebplanner::LineObstacle>((*m_obstacles)[m_selected_index]);
                    l_obstacle->setEnd(rp);
                    break;
                }
                case SelectedType::kLine:
                {
                    std::shared_ptr<navigation::tebplanner::LineObstacle> l_obstacle = std::dynamic_pointer_cast<navigation::tebplanner::LineObstacle>((*m_obstacles)[m_selected_index]);
                    l_obstacle->setCentroid(rp);
                    break;
                }
                case SelectedType::kPolygonPoint:
                {
                    std::shared_ptr<navigation::tebplanner::PolygonObstacle> p_obstacle = std::dynamic_pointer_cast<navigation::tebplanner::PolygonObstacle>((*m_obstacles)[m_selected_index]);
                    p_obstacle->setPoint(m_selected_polygon_index, rp);
                    break;
                }
                case SelectedType::kPolygon:
                {
                    std::shared_ptr<navigation::tebplanner::PolygonObstacle> p_obstacle = std::dynamic_pointer_cast<navigation::tebplanner::PolygonObstacle>((*m_obstacles)[m_selected_index]);
                    p_obstacle->setCentroid(rp);
                    break;
                }
                }
                m_position_widget->setPosition(rp);
                m_position_widget->move(QPoint(x, y));
                update();
            }

            void PathInnerWidget::mouseReleaseEvent(QMouseEvent* event)
            {
            }

            QMatrix PathInnerWidget::getTransform(double* s) const
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

            void PathInnerWidget::drawBoundingBox(QPainter& painter, double scale)
            {
                painter.setPen(QPen(Qt::gray, scale));
                painter.setBrush(Qt::white);

                painter.drawRect(m_bounding_box.minX()(), m_bounding_box.minY()(), m_bounding_box.width()(), m_bounding_box.height()());
            }

            void PathInnerWidget::drawStartAndGoal(QPainter& painter, double scale)
            {
                drawPose(*m_start, Qt::green, painter, scale);
                drawPose(*m_goal, Qt::red, painter, scale);
            }

            void PathInnerWidget::drawPose(const navigation::tebplanner::Pose& pose, Qt::GlobalColor color, QPainter& painter, double scale)
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

            void PathInnerWidget::showContextMenu(const QPoint& point)
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

            bool PathInnerWidget::checkPointClick(const geometry::Point& point, const QMatrix& transform, const QPoint& mouse_pos, PathInnerWidget::SelectedType select, int index, int polygon_index, unsigned int pixel_threshold)
            {
                QPoint screen_point(point.x()(), point.y()());
                screen_point = screen_point * transform;
                if ((screen_point - mouse_pos).manhattanLength() < pixel_threshold)
                {
                    int x = std::max(std::min(mouse_pos.x() - 90, width() - 250), 75);
                    int y = std::max(std::min(mouse_pos.y() + 17, height() - 115), 75);

                    m_selected_type = select;
                    m_selected_index = index;
                    m_selected_polygon_index = polygon_index;

                    m_position_widget = std::unique_ptr<PositionWidget>(new PositionWidget(point, this));
                    m_position_widget->move(QPoint(x, y));
                    m_position_widget->show();
                    connect(m_position_widget.get(), SIGNAL(updatePosition()), this, SLOT(updateSelectedPosition()));
                    return true;
                }
                return false;
            }

            void PathInnerWidget::drawObstacles(QPainter& painter, double scale)
            {
                if (!m_obstacles || m_obstacles->empty())
                {
                    return;
                }

                painter.setPen(QPen(Qt::black, scale));
                painter.setBrush(Qt::black);

                int diameter = scale * 5;

                for (std::shared_ptr< navigation::tebplanner::Obstacle > obstacle : (*m_obstacles))
                {
                    std::shared_ptr<navigation::tebplanner::PointObstacle> point = std::dynamic_pointer_cast<navigation::tebplanner::PointObstacle>(obstacle);
                    if (point)
                    {
                        painter.drawEllipse(point->getCentroidPoint().x()() - diameter / 2.0, point->getCentroidPoint().y()() - diameter / 2.0, diameter, diameter);

                        continue;
                    }

                    std::shared_ptr< navigation::tebplanner::LineObstacle > line = std::dynamic_pointer_cast< navigation::tebplanner::LineObstacle >(obstacle);
                    if (line)
                    {
                        painter.drawLine(line->startPoint().x()(), line->startPoint().y()(), line->endPoint().x()(), line->endPoint().y()());
                        continue;
                    }

                    std::shared_ptr< navigation::tebplanner::PolygonObstacle > polygon = std::dynamic_pointer_cast< navigation::tebplanner::PolygonObstacle >(obstacle);
                    if (polygon)
                    {
                        QPainterPath path;
                        geometry::Polygon poly = polygon->polygon();
                        geometry::Point p = poly.front();
                        path.moveTo(p.x()(), p.y()());
                        for (int i = 1; i <= poly.size(); i++)
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

            void PathInnerWidget::drawTrajectory(QPainter& painter, double scale)
            {
                if (m_trajectory.empty())
                {
                    return;
                }

                painter.setPen(QPen(Qt::blue, scale));
                painter.setBrush(Qt::blue);

                bool first = true;
                geometry::Point p0 = m_trajectory.front().position();

                double diameter = scale * 5;

                for (const navigation::tebplanner::TrajectoryPoint& point : m_trajectory)
                {
                    painter.drawEllipse(p0.x()() - diameter / 2.0, p0.y()() - diameter / 2.0, diameter, diameter);

                    if (first)
                    {
                        first = false;
                        continue;
                    }

                    geometry::Point p1 = point.position();

                    painter.drawLine(p0.x()(), p0.y()(), p1.x()(), p1.y()());
                    p0 = p1;
                }
            }

            void PathInnerWidget::drawWaypoints(QPainter& painter, double scale)
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

            void PathInnerWidget::drawRobot(QPainter& painter, double scale)
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

                    std::shared_ptr< navigation::tebplanner::PointRobotFootprint > point = std::dynamic_pointer_cast< navigation::tebplanner::PointRobotFootprint >(m_robot);
                    if (point)
                    {
                        int diameter = scale * 5;
                        painter.drawEllipse(position.x()() - diameter / 2.0, position.y()() - diameter / 2.0, diameter, diameter);
                    }

                    std::shared_ptr< navigation::tebplanner::CircularRobotFootprint > circle = std::dynamic_pointer_cast< navigation::tebplanner::CircularRobotFootprint >(m_robot);
                    if (circle)
                    {
                        int diameter = (circle->getInscribedRadius() * units::m)() * 2;
                        painter.drawEllipse(position.x()() - diameter / 2.0, position.y()() - diameter / 2.0, diameter, diameter);
                    }

                    std::shared_ptr< navigation::tebplanner::PolygonRobotFootprint > polygon = std::dynamic_pointer_cast< navigation::tebplanner::PolygonRobotFootprint >(m_robot);
                    if (polygon)
                    {
                        geometry::Polygon poly = polygon->polygon(position, rotation);

                        QPainterPath path;

                        geometry::Point p = poly.front();
                        path.moveTo(p.x()(), p.y()());
                        for (int i = 1; i < poly.size(); i++)
                        {
                            p = poly[i];
                            path.lineTo(p.x()(), p.y()());
                        }
                        painter.drawPath(path);
                    }
                }

            }

            void PathInnerWidget::createBoundingBox()
            {
                units::Distance minimum = -1 * units::in;
                units::Distance maximum = 1 * units::in;

                if (m_start)
                {
                    minimum = units::min(m_start->position().x(), m_start->position().y(), minimum);
                    maximum = units::max(m_start->position().x(), m_start->position().y(), maximum);
                }

                if (m_goal)
                {
                    minimum = units::min(m_goal->position().x(), m_goal->position().y(), minimum);
                    maximum = units::max(m_goal->position().x(), m_goal->position().y(), maximum);
                }

                for (const geometry::Point& point : m_waypoints)
                {
                    minimum = units::min(point.x(), point.y(), minimum);
                    maximum = units::max(point.x(), point.y(), maximum);
                }

                if(m_obstacles)
                {
                    for (std::shared_ptr< navigation::tebplanner::Obstacle > obstacle : (*m_obstacles))
                    {
                        std::shared_ptr< navigation::tebplanner::PointObstacle > point = std::dynamic_pointer_cast< navigation::tebplanner::PointObstacle >(obstacle);
                        if (point)
                        {
                            minimum = units::min(point->getCentroidPoint().x(), point->getCentroidPoint().y(), minimum);
                            maximum = units::max(point->getCentroidPoint().x(), point->getCentroidPoint().y(), maximum);
                            continue;
                        }

                        std::shared_ptr< navigation::tebplanner::LineObstacle > line = std::dynamic_pointer_cast< navigation::tebplanner::LineObstacle >(obstacle);
                        if (line)
                        {
                            minimum = units::min(line->startPoint().x(), line->startPoint().y(), line->endPoint().x(), line->endPoint().y(), minimum);
                            maximum = units::max(line->startPoint().x(), line->startPoint().y(), line->endPoint().x(), line->endPoint().y(), maximum);
                            continue;
                        }

                        std::shared_ptr< navigation::tebplanner::PolygonObstacle> polygon = std::dynamic_pointer_cast< navigation::tebplanner::PolygonObstacle >(obstacle);
                        if (polygon)
                        {
                            for (const geometry::Point& point : polygon->polygon())
                            {
                                minimum = units::min(point.x(), point.y(), minimum);
                                maximum = units::max(point.x(), point.y(), maximum);
                            }
                        }
                    }
                }

                m_bounding_box = geometry::Rectangle(minimum, maximum, minimum, maximum);
            }

        }
    }
}
