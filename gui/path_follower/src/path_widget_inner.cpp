#include "path_follower_gui/path_widget_inner.hpp"

#include <QPaintEvent>
#include <QResizeEvent>
#include <QAction>
#include <QMenu>

#include "path_follower_gui/storage.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            namespace gui
            {
                PathWidgetInner::PathWidgetInner(QWidget* parent)
                    : QWidget(parent)
                    , m_position_widget(nullptr)
                    , m_bounding_box(-1 * units::in, 1 * units::in, -1 * units::in, 1 * units::in)
                    , m_robot(nullptr)
                    , m_world(nullptr)
                    , m_waypoints(nullptr)
                    , m_selected(false)
                    , m_selected_index(-1)
                {
                    connect(Storage::getInstance().get(), SIGNAL(selectedWorldChanged()), this, SLOT(updateWorld()));
                    connect(Storage::getInstance().get(), SIGNAL(selectedRobotChanged()), this, SLOT(updatRobot()));
                    connect(Storage::getInstance().get(), SIGNAL(selectedWaypointsChanged()), this, SLOT(updateWaypoints()));
                }

                void PathWidgetInner::updateWorld()
                {
                    m_world = Storage::getInstance()->selectedWorld();
                    updateGeometry();
                    update();
                }

                void PathWidgetInner::updateRobot()
                {
                    m_robot = Storage::getInstance()->selectedRobot();
                    update();
                }

                void PathWidgetInner::updateWaypointList()
                {
                    m_waypoints = Storage::getInstance()->selectedWaypoints();
                    update();
                }

                void PathWidgetInner::paintEvent(QPaintEvent* event)
                {
                    if(!m_world)
                    {
                        return;
                    }
                    m_bounding_box = m_world->boundary().boundingBox();

                    QPainter painter;
                    painter.begin(this);
                    painter.setRenderHint(QPainter::Antialiasing);
                    painter.fillRect(event->rect(), Qt::black);

                    double s;
                    QMatrix transform = getTransform(&s);
                    painter.setTransform(QTransform(transform));
                    painter.save();

                    s /= 250;
                    m_world->draw(painter);

                    // todo: painter robot and path

                    if(m_waypoints)
                    {
                        m_waypoints->draw(painter, s);
                        if(m_robot)
                        {
                            //m_robot->draw(painter, );
                        }
                    }

                    painter.restore();
                    painter.end();
                }

                void PathWidgetInner::resizeEvent(QResizeEvent* event)
                {
                    if (!m_world)
                    {
                        QWidget::resizeEvent(event);
                        return;
                    }
                    m_bounding_box = m_world->boundary().boundingBox();

                    int given_w = event->size().width();
                    int given_h = event->size().height();

                    int needed_w, needed_h;

                    units::Distance min_x, max_x, min_y, max_y;

                    geometry::Point center = m_bounding_box.center();
                    min_x = center.x() - (center.x() - m_bounding_box.minX()) * 1.2;
                    max_x = center.x() + (m_bounding_box.maxX() - center.x()) * 1.2;
                    min_y = center.y() - (center.y() - m_bounding_box.minY()) * 1.2;
                    max_y = center.y() + (m_bounding_box.maxY() - center.y()) * 1.2;

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

                void PathWidgetInner::mousePressEvent(QMouseEvent* event)
                {
                    if (event->button() == Qt::RightButton)
                    {
                        if (m_robot)
                        {
                            showContextMenu(event->pos());
                        }
                        return;
                    }

                    QPoint pos = event->pos();

                    double s;
                    QMatrix transform = getTransform(&s);

                    // If click is outside the bounding box then short circuit
                    QMatrix inverse_transform = transform.inverted();
                    QPoint rp = pos * inverse_transform;
                    geometry::Point real_point(rp.x(), rp.y());
                    /*
                    if (m_bounding_box.clip(real_point) != real_point)
                    {
                        return;
                    }
                    */

                    s /= 250;
                    if (m_waypoints)
                    {
                        int x = std::max(std::min(pos.x() - 90, width() - 250), 75);
                        int y = std::max(std::min(pos.y() + 17, height()  - 115), 75);
                        std::vector<Waypoint> waypoints = m_waypoints->waypoints();
                        for (size_t i = 0; i < waypoints.size(); i++)
                        {
                            if (checkPointClick(geometry::Point(waypoints[i].x(), waypoints[i].y()), transform, pos, i))
                            {
                                return;
                            }
                        }
                    }

                    if (m_position_widget)
                    {
                        m_position_widget->hide();
                        disconnect(m_position_widget.get(), SIGNAL(updatePosition()), this, SLOT(updateSelectedPosition()));
                        m_position_widget.reset();
                        m_selected = false;
                    }

                }

                void PathWidgetInner::mouseMoveEvent(QMouseEvent* event)
                {

                }

                QMatrix PathWidgetInner::getTransform(double* s) const
                {
                    QMatrix transform;
                    transform.translate(width() / 2.0, height() / 2.0);

                    units::Distance min_x, max_x, min_y, max_y;

                    geometry::Point center = m_bounding_box.center();
                    min_x = center.x() - (center.x() - m_bounding_box.minX()) * 1.2;
                    max_x = center.x() + (m_bounding_box.maxX() - center.x()) * 1.2;
                    min_y = center.y() - (center.y() - m_bounding_box.minY()) * 1.2;
                    max_y = center.y() + (m_bounding_box.maxY() - center.y()) * 1.2;

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

                bool PathWidgetInner::checkPointClick(const geometry::Point& point, const QMatrix& transform, const QPoint& mouse_pos, int index, unsigned int pixel_theshold)
                {
                    QPoint screen_point(point.x()(), point.y()());
                    screen_point = screen_point * transform;
                    if ((screen_point - mouse_pos).manhattanLength() < pixel_theshold)
                    {
                        int x = std::max(std::min(mouse_pos.x() - 90, width() - 250), 75);
                        int y = std::max(std::min(mouse_pos.y() + 17, height() - 115), 75);

                        m_selected = true;
                        m_selected_index = index;

                        m_position_widget = std::unique_ptr<PositionWidget>(new PositionWidget(point, this));
                        m_position_widget->move(QPoint(x, y));
                        m_position_widget->show();
                        connect(m_position_widget.get(), SIGNAL(updatePosition()), this, SLOT(updateSelectedPosition()));
                        return true;
                    }
                    return false;
                }

                void PathWidgetInner::showContextMenu(const QPoint& point)
                {
                    QMenu context_menu(tr("Context menu"), this);

                    QAction* action;

                    // todo(Andrew): edit number of vertices

                    /*
                    action = new QAction("Add Obstacle", this);
                    action->setIcon(QIcon(":/Icons/add.png"));
                    connect(action, SIGNAL(triggered(bool)), this, SLOT(addObstacle()));
                    context_menu.addAction(action);
                    */

                    context_menu.exec(mapToGlobal(point));
                }
            }
        }
    }
}
