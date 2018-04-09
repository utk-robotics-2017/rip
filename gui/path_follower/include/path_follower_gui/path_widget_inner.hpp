#ifndef PATH_WIDGET_INNER_HPP
#define PATH_WIDGET_INNER_HPP

#include <QWidget>

#include <memory>

#include <geometry/rectangle.hpp>
#include <geometry/point.hpp>
#include <path_follower/path.hpp>
#include "path_follower_gui/world.hpp"
#include "path_follower_gui/robot.hpp"
#include "path_follower_gui/waypoint_list.hpp"
#include "path_follower_gui/position_widget.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            namespace gui
            {
                class PathWidgetInner : public QWidget
                {
                    Q_OBJECT
                public:
                    explicit PathWidgetInner(QWidget* parent = nullptr);

                private slots:
                    void updateSelectedPosition();

                public slots:
                    void updateWorld();
                    void updateRobot();
                    void updateWaypointList();

                protected:
                    /**
                     * Draws the 2D widget
                     */
                    virtual void paintEvent(QPaintEvent* event) override;

                    /**
                     * Resizes the widget
                     */
                    virtual void resizeEvent(QResizeEvent* event) override;

                    /**
                     * Handle mouse click events
                     */
                    virtual void mousePressEvent(QMouseEvent* event) override;

                    /**
                     * Handle mouse drag events
                     */
                    virtual void mouseMoveEvent(QMouseEvent* event) override;

                private:
                    QMatrix getTransform(double* s = nullptr) const;

                    bool checkPointClick(const geometry::Point& point, const QMatrix& transform, const QPoint& mouse_pos, int index, unsigned int pixel_theshold = 15);


                    /**
                     * Show the context menu when right clicked
                     */
                    void showContextMenu(const QPoint& point);

                    std::shared_ptr<World> m_world;
                    std::shared_ptr<Robot> m_robot;
                    std::shared_ptr<WaypointList> m_waypoints;
                    geometry::Rectangle m_bounding_box;
                    bool m_selected;
                    int m_selected_index;
                    std::unique_ptr<PositionWidget> m_position_widget;
                };
            }
        }
    }
}

#endif // PATH_WIDGET_INNER_HPP
