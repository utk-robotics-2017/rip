#include "path_follower_gui/world_widget_inner.hpp"

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
                WorldWidgetInner::WorldWidgetInner(QWidget* parent)
                    : QWidget(parent)
                    , m_position_widget(nullptr)
                    , m_bounding_box(-1 * units::in, 1 * units::in, -1 * units::in, 1 * units::in)
                    , m_world(nullptr)
                    , m_selected_type(SelectedType::kNone)
                    , m_selected_index(-1)
                    , m_selected_polygon_index(-1)
                {
                    connect(Storage::getInstance().get(), SIGNAL(selectedWorldChanged()), this, SLOT(updateWorld()));
                }

                void WorldWidgetInner::updateWorld()
                {
                    m_world = Storage::getInstance()->selectedWorld();
                    if(m_world)
                    {
                        if(m_world->boundary().size() == 0)
                        {
                            bool ok;
                            int n_vertices = QInputDialog::getInt(this, "New World", "Number of vertices:", 3, 3, 2147483647, 1, &ok);
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
                                m_world->setBoundary(points);
                            }
                        }
                    }
                    updateGeometry();
                    update();
                }

                void WorldWidgetInner::updateSelectedPosition()
                {
                    double scale;
                    QMatrix transform = getTransform(&scale);
                    QPoint screen_point;
                    std::shared_ptr<Obstacle> obstacle;
                    switch (m_selected_type)
                    {
                    case SelectedType::kNone:
                    {
                        return; // Shouldn't happen...
                    }
                    case SelectedType::kObstaclePoint:
                    {
                        obstacle = std::dynamic_pointer_cast<Obstacle>(m_world->obstacle(m_selected_index));
                        obstacle->setPoint(m_selected_polygon_index, m_position_widget->position());
                        screen_point = QPoint(obstacle->polygon()[m_selected_polygon_index].x()(), obstacle->polygon()[m_selected_polygon_index].y()());
                        break;
                    }
                    case SelectedType::kObstacle:
                    {
                        obstacle = std::dynamic_pointer_cast<Obstacle>(m_world->obstacle(m_selected_index));
                        obstacle->setCenter(m_position_widget->position());
                        screen_point = QPoint(obstacle->center().x()(), obstacle->center().y()());
                        break;
                    }
                    case SelectedType::kBoundaryPoint:
                    {
                        m_world->setBoundaryPoint(m_selected_index, m_position_widget->position());
                        screen_point = QPoint(m_world->boundary()[m_selected_index].x()(), m_world->boundary()[m_selected_index].y()());
                        m_bounding_box = m_world->boundary().boundingBox();
                        break;
                    }
                    }
                    screen_point = screen_point * transform;
                    int x = std::max(std::min(screen_point.x() - 45, width()-250), 100);
                    int y = std::max(std::min(screen_point.y() + 17, height() - 115), 60);
                    m_position_widget->move(QPoint(x,y));
                    updateGeometry();
                    update();
                }

                void WorldWidgetInner::addObstacle()
                {
                    bool ok;
                    int n_vertices = QInputDialog::getInt(this, "Add Obstacle", "Number of vertices:", 3, 3, 2147483647, 1, &ok);
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
                        m_world->addObstacle(std::make_shared<Obstacle>(points));
                        updateGeometry();
                        update();
                    }
                }

                void WorldWidgetInner::paintEvent(QPaintEvent* event)
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
                    painter.restore();
                    painter.end();
                }

                void WorldWidgetInner::resizeEvent(QResizeEvent* event)
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

                void WorldWidgetInner::mousePressEvent(QMouseEvent* event)
                {
                    if (event->button() == Qt::RightButton)
                    {
                        if (m_world)
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

                    if (m_world)
                    {
                        int x = std::max(std::min(pos.x() - 90, width() - 250), 75);
                        int y = std::max(std::min(pos.y() + 17, height()  - 115), 75);
                        if(m_world->numObstacles())
                        {
                            for (size_t idx = 0; idx < m_world->numObstacles(); ++idx)
                            {
                                std::shared_ptr<Obstacle> obstacle = m_world->obstacle(idx);
                                if (obstacle)
                                {
                                    geometry::Polygon polygon = obstacle->polygon();
                                    for (size_t i = 0; i < polygon.size(); i++)
                                    {
                                        if (checkPointClick(polygon[i], transform, pos, SelectedType::kObstaclePoint, idx, i))
                                        {
                                            return;
                                        }
                                    }

                                    if (checkPointClick(polygon.centroid(), transform, pos, SelectedType::kObstacle, idx))
                                    {
                                        return;
                                    }

                                    if (polygon.inside(real_point))
                                    {
                                        m_selected_type = SelectedType::kObstacle;
                                        m_selected_index = idx;

                                        m_position_widget = std::unique_ptr<PositionWidget>(new PositionWidget(polygon.centroid(), this));
                                        m_position_widget->move(QPoint(x, y));
                                        m_position_widget->show();
                                        connect(m_position_widget.get(), SIGNAL(updatePosition()), this, SLOT(updateSelectedPosition()));
                                        return;
                                    }
                                }
                            }
                        }

                        geometry::Polygon polygon = m_world->boundary();
                        for (size_t i = 0; i < polygon.size(); i++)
                        {
                            if (checkPointClick(polygon[i], transform, pos, SelectedType::kBoundaryPoint, i))
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
                        m_selected_type = SelectedType::kNone;
                    }
                }

                void WorldWidgetInner::mouseMoveEvent(QMouseEvent* event)
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
                        throw SingularTransformException();
                    }

                    QPoint real_point = pos * inverse_transform;
                    geometry::Point rp = geometry::Point(real_point.x(), real_point.y());

                    std::shared_ptr<Obstacle> obstacle;
                    switch (m_selected_type)
                    {
                    case SelectedType::kObstacle:
                        obstacle = m_world->obstacle(m_selected_index);
                        if (obstacle)
                        {
                            obstacle->setCenter(rp);
                        }
                        else
                        {
                            throw SelectedObstacleMissing("The selected obstacle no longer exist");
                            // I do not know how you did this...
                        }
                        break;
                    case SelectedType::kObstaclePoint:
                        obstacle = m_world->obstacle(m_selected_index);
                        if (obstacle)
                        {
                            obstacle->setPoint(m_selected_polygon_index, rp);
                        }
                        else
                        {
                            throw SelectedObstacleMissing("The selected obstacle no longer exist");
                            // I do not know how you did this...
                        }
                        break;
                    case SelectedType::kBoundaryPoint:
                        m_world->setBoundaryPoint(m_selected_index, rp);
                        m_bounding_box = m_world->boundary().boundingBox();
                        break;
                    default:
                        throw UnknownSelectedType("Selected type is not possible for the world widget");
                    }

                    m_position_widget->setPosition(rp);
                    m_position_widget->move(QPoint(x, y));
                    updateGeometry();
                    update();
                }

                QMatrix WorldWidgetInner::getTransform(double* s) const
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

                bool WorldWidgetInner::checkPointClick(const geometry::Point& point, const QMatrix& transform, const QPoint& mouse_pos, WorldWidgetInner::SelectedType select, int index, int polygon_index, unsigned int pixel_theshold)
                {
                    QPoint screen_point(point.x()(), point.y()());
                    screen_point = screen_point * transform;
                    if ((screen_point - mouse_pos).manhattanLength() < pixel_theshold)
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

                void WorldWidgetInner::showContextMenu(const QPoint& point)
                {
                    QMenu context_menu(tr("Context menu"), this);

                    QAction* action;

                    action = new QAction("Add Obstacle", this);
                    action->setIcon(QIcon(":/Icons/add.png"));
                    connect(action, SIGNAL(triggered(bool)), this, SLOT(addObstacle()));
                    context_menu.addAction(action);

                    //context_menu.addSeparator();

                    // todo: delete if right click on point

                    context_menu.exec(mapToGlobal(point));
                }
            }
        }
    }
}
