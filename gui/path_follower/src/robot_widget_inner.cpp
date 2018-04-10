#include "path_follower_gui/robot_widget_inner.hpp"

#include <QResizeEvent>
#include <QAction>
#include <QMenu>
#include <QInputDialog>

#include "path_follower_gui/storage.hpp"
#include "path_follower_gui/exceptions.hpp"
namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            namespace gui
            {
                RobotWidgetInner::RobotWidgetInner(QWidget* parent)
                    : QWidget(parent)
                    , m_position_widget(nullptr)
                    , m_bounding_box(-1 * units::in, 1 * units::in, -1 * units::in, 1 * units::in)
                    , m_robot(nullptr)
                    , m_selected(false)
                    , m_selected_index(-1)
                {

                    connect(Storage::getInstance().get(), SIGNAL(selectedRobotChanged()), this, SLOT(updateRobot()));
                }

                void RobotWidgetInner::updateRobot()
                {
                    m_robot = Storage::getInstance()->selectedRobot();
                    if(m_robot)
                    {
                        if(m_robot->shape().size() == 0)
                        {
                            bool ok;
                            int n_vertices = QInputDialog::getInt(this, "New Robot", "Number of vertices:", 4, 3, 2147483647, 1, &ok);
                            if (ok)
                            {
                                double s;
                                getTransform(&s);
                                s /= 250;

                                std::vector< geometry::Point > points;
                                units::Angle step = 360 / n_vertices * units::deg;
                                for (int i = 0; i < n_vertices; i++)
                                {
                                    points.emplace_back(units::cos(i * step), units::sin(i * step));
                                    points.back() *= 15 * s;
                                }
                                m_robot->setShape(points);
                            }
                        }
                        updateGeometry();
                        update();
                    }
                }

                void RobotWidgetInner::updateSelectedPosition()
                {
                    if (!m_selected)
                    {
                        return; // Shouldn't happen...
                    }
                    double scale;
                    QMatrix transform = getTransform(&scale);
                    QPoint screen_point;
                    m_robot->setPoint(m_selected_index, m_position_widget->position());
                    screen_point = QPoint(m_robot->shape()[m_selected_index].x()(), m_robot->shape()[m_selected_index].y()());

                    screen_point = screen_point * transform;
                    int x = std::max(std::min(screen_point.x() - 45, width() - 250), 100);
                    int y = std::max(std::min(screen_point.y() + 17, height() - 115), 60);
                    m_position_widget->move(QPoint(x, y));
                    update();
                }

                void RobotWidgetInner::paintEvent(QPaintEvent* event)
                {
                    if (!m_robot)
                    {
                        return;
                    }
                    m_bounding_box = m_robot->shape().boundingBox();

                    QPainter painter;
                    painter.begin(this);
                    painter.setRenderHint(QPainter::Antialiasing);
                    painter.fillRect(event->rect(), Qt::black);

                    double s;
                    QMatrix transform = getTransform(&s);
                    painter.setTransform(QTransform(transform));
                    painter.save();

                    s /= 250;
                    m_robot->draw(painter, RigidTransform2d());
                    painter.restore();
                    painter.end();
                }

                void RobotWidgetInner::resizeEvent(QResizeEvent* event)
                {
                    if (!m_robot)
                    {
                        QWidget::resizeEvent(event);
                        return;
                    }
                    m_bounding_box = m_robot->shape().boundingBox();

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

                void RobotWidgetInner::mousePressEvent(QMouseEvent* event)
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
                    if (m_robot)
                    {
                        int x = std::max(std::min(pos.x() - 90, width() - 250), 75);
                        int y = std::max(std::min(pos.y() + 17, height()  - 115), 75);
                        geometry::Polygon polygon = m_robot->shape();
                        for (size_t i = 0; i < polygon.size(); i++)
                        {
                            if (checkPointClick(polygon[i], transform, pos, i))
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

                void RobotWidgetInner::mouseMoveEvent(QMouseEvent* event)
                {
                    if (event->button() == Qt::RightButton || !m_selected)
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
                        throw SingularTransformException();
                    }

                    QPoint real_point = pos * inverse_transform;
                    geometry::Point rp = geometry::Point(real_point.x(), real_point.y());

                    m_robot->setPoint(m_selected_index, rp);

                    m_position_widget->setPosition(rp);
                    m_position_widget->move(QPoint(x, y));
                    update();
                }

                QMatrix RobotWidgetInner::getTransform(double* s) const
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

                bool RobotWidgetInner::checkPointClick(const geometry::Point& point, const QMatrix& transform, const QPoint& mouse_pos, int index, unsigned int pixel_theshold)
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

                void RobotWidgetInner::showContextMenu(const QPoint& point)
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
