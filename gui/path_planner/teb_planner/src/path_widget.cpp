#include <teb_planner_gui/path_widget.hpp>

#include <QMenu>
#include <QAction>
#include <QPainterPath>

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
                , m_bounding_box(-1 * units::in, 1 * units::in, 1 * units::in, -1 * units::in)
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

            void PathWidget::paintEvent(QPaintEvent* event)
            {
                QPainter painter;
                painter.begin(this);
                painter.setRenderHint(QPainter::Antialiasing);
                painter.fillRect(event->rect(), Qt::white);

                painter.translate(width() / 2.0, height() / 2.0);

                units::Distance min_x, max_x, min_y, max_y;

                geometry::Point center = m_bounding_box.center();
                min_x = center.x() - (center.x() - m_bounding_box.minX()) * 1.2;
                max_x = center.x() + (m_bounding_box.maxX() - center.x()) * 1.2;
                min_y = center.y() - (center.y() - m_bounding_box.minY()) * 1.2;
                max_y = center.y() + (m_bounding_box.maxY() - center.x()) * 1.2;

                geometry::Point scale(max_x - min_x, max_y - min_y);
                painter.scale(width(), -height());
                painter.scale( 1.0 / scale.x()(), 1.0 / scale.y()());

                painter.translate(-center.x()(), -center.y()());

                painter.save();

                double s = scale.x()() / 250;

                drawObstacles(painter, s);
                drawTrajectory(painter, s);
                // todo: draw start and goal
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

                QMatrix transform;
                transform.translate(width() / 2.0, height() / 2.0);

                units::Distance min_x, max_x, min_y, max_y;

                geometry::Point center = m_bounding_box.center();
                min_x = center.x() - (center.x() - m_bounding_box.minX()) * 1.2;
                max_x = center.x() + (m_bounding_box.maxX() - center.x()) * 1.2;
                min_y = center.y() - (center.y() - m_bounding_box.minY()) * 1.2;
                max_y = center.y() + (m_bounding_box.maxY() - center.x()) * 1.2;

                geometry::Point scale(max_x - min_x, max_y - min_y);
                transform.scale(width(), -height());
                transform.scale( 1.0 / scale.x()(), 1.0 / scale.y()());

                transform.translate(-center.x()(), -center.y()());

                int pixel_threshold = 5;

                QPoint screen_point;

                if(m_start)
                {
                    screen_point = QPoint(m_start->position().x()(), m_start->position().y()());
                    screen_point = screen_point * transform;
                    // Within 5 pixels
                    if((screen_point - pos).manhattanLength() < pixel_threshold)
                    {
                        m_selected_type = SelectedType::kStart;

                        m_position_widget = std::unique_ptr<PositionWidget>(new PositionWidget(m_start->position(), this));
                        m_position_widget->move(QPoint(screen_point.x() - 45, screen_point.y() + 17));
                        m_position_widget->show();
                        return;
                    }
                }
                /*
                screen_point = QPoint(goal.position().x()(), goal.position().y()());
                screen_point *= transform;
                if((screen_point - pos).manhattanLength() < pixel_threshold)
                {
                    m_selected_type = SelectedType::kGoal;


                    // todo: display labels and textboxes
                    return;
                }

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
                }
            }

            void PathWidget::showContextMenu(const QPoint& point)
            {
                QMenu context_menu(tr("Context menu"), this);

                QAction* action;

                action = new QAction("Add Start", this);
                action->setIcon(QIcon(":/Icons/add.png"));
                context_menu.addAction(action);

                action = new QAction("Add Goal", this);
                action->setIcon(QIcon(":/Icons/add.png"));
                context_menu.addAction(action);

                action = new QAction("Add Obstacle", this);
                action->setIcon(QIcon(":/Icons/add.png"));
                context_menu.addAction(action);

                context_menu.addSeparator();

                // todo: delete if right click on point

                context_menu.exec(mapToGlobal(point));
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
