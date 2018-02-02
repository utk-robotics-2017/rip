#ifndef ROBOT_INNER_WIDGET_HPP
#define ROBOT_INNER_WIDGET_HPP

#include <memory>

#include <QWidget>
#include <QResizeEvent>
#include <QPaintEvent>

#include <geometry/polygon.hpp>
#include <teb_planner/robot_footprint_model.hpp>

#include "teb_planner_gui/settings.hpp"
#include "teb_planner_gui/position_widget.hpp"

namespace rip
{
    namespace gui
    {
        namespace tebplanner
        {
            class RobotInnerWidget : public QWidget
            {
                Q_OBJECT
            public:
                explicit RobotInnerWidget(QWidget* parent = nullptr);

                void setRobot(std::shared_ptr<navigation::tebplanner::PolygonRobotFootprint> robot);

            private slots:
                /**
                 * Handle mouse click events
                 */
                virtual void mousePressEvent(QMouseEvent* event) override;

                /**
                 * Handle mouse drag events
                 */
                virtual void mouseMoveEvent(QMouseEvent* event) override;

                void updateNumberOfPoints();

                void updateSelectedPosition();

            protected:
                /**
                 * "Paint" the screen
                 */
                virtual void paintEvent(QPaintEvent* event) override;

                /**
                 * Handle resizing
                 */
                virtual void resizeEvent(QResizeEvent* event) override;

            private:
                /**
                 * Draw the bounding box
                 */
                void drawBoundingBox(QPainter& painter, const double& scale);

                /**
                 * Draw the robot on the screen
                 */
                void drawRobot(QPainter& painter, const double& scale);

                /**
                 * Calculates the axis aligned bounding box
                 */
                void createBoundingBox();

                /**
                 * Returns the transform from the real world to the screen world
                 *
                 * @param s The scale transform (can be used for line widths and point diameters)
                 */
                QMatrix getTransform(double* s = nullptr) const;

                /**
                 * Show the context menu when right clicked
                 */
                void showContextMenu(const QPoint& point);

                bool checkPointClick(const geometry::Point& point, const QMatrix& transform, const QPoint& mouse_pos, int index, unsigned int pixel_threshold = 15);


                geometry::Rectangle m_bounding_box;
                std::shared_ptr<navigation::tebplanner::PolygonRobotFootprint> m_robot;
                std::shared_ptr<Settings> m_settings;

                bool m_selected;
                int m_selected_index;
                std::unique_ptr<PositionWidget> m_position_widget;
            };
        }
    }
}

#endif // ROBOT_INNER_WIDGET_HPP
