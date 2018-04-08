#ifndef ROBOT_WIDGET_INNER_HPP
#define ROBOT_WIDGET_INNER_HPP

#include <memory>

#include <QWidget>

#include "path_follower_gui/robot.hpp"
#include "path_follower_gui/position_widget.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            namespace gui
            {
                class RobotWidgetInner : public QWidget
                {
                    Q_OBJECT
                public:
                    explicit RobotWidgetInner(QWidget* parent);

                    void setRobot(std::shared_ptr<Robot> robot, bool added = false);
                private slots:
                    void updateSelectedPosition();

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

                    std::shared_ptr<Robot> m_robot;
                    geometry::Rectangle m_bounding_box;
                    bool m_selected;
                    int m_selected_index;
                    std::unique_ptr<PositionWidget> m_position_widget;
                };
            }
        }
    }
}

#endif // ROBOT_WIDGET_INNER_HPP
