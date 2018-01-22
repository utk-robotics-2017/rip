#ifndef PATH_WIDGET_HPP
#define PATH_WIDGET_HPP

#include <memory>
#include <vector>

#include <QWidget>
#include <QPainter>
#include <QPaintEvent>
#include <QResizeEvent>
#include <QTimer>

#include "teb_planner/robot_footprint_model.hpp"
#include "teb_planner/obstacle.hpp"
#include "teb_planner/trajectory_point.hpp"

#include "position_widget.hpp"

namespace rip
{
    namespace gui
    {
        namespace tebplanner
        {
            /**
             * A widget that draws the start position, goal position, waypoints, the trajectory and the robot following
             * the trajectory. Adding new obstacles and waypoints is also possible
             */
            class PathWidget : public QWidget
            {
                Q_OBJECT
            public:
                /**
                 * Constructor
                 */
                explicit PathWidget(QWidget* parent = nullptr);

                /**
                 * Sets whether the robot should be animated
                 */
                void setAnimate(bool animate);

                void setStart(const navigation::Pose& start);

                void setGoal(const navigation::Pose& goal);

                /**
                 * Set the robot footprint model
                 */
                void setRobot(std::shared_ptr< navigation::RobotFootprintModel > robot);

                /**
                 * Sets the obstacles
                 */
                void setObstacles(const std::vector< std::shared_ptr<navigation::Obstacle> >& obstacles);

            protected:
                /**
                 * Paints the widget
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
            private:
                /**
                 * Draw the obstacles
                 */
                void drawObstacles(QPainter& painter, double scale);

                /**
                 * Draw the trajectory
                 */
                void drawTrajectory(QPainter& painter, double scale);

                /**
                 * Draw the waypoints
                 */
                void drawWaypoints(QPainter& painter, double scale);

                /**
                 * Draw the robot as it traverses the path
                 */
                void drawRobot(QPainter& painter, double scale);

                /**
                 * Create the bounding box for the obstacles
                 */
                void createBoundingBox();

                /**
                 * Show the context menu when right clicked
                 */
                void showContextMenu(const QPoint& point);

                enum class SelectedType
                {
                    kStart,
                    kGoal,
                    kWaypoint,
                    kPoint,
                    kLineStart,
                    kLineEnd,
                    kPolygon
                };

                SelectedType m_selected_type;
                int m_selected_index;

                bool m_animate;
                int m_timestep;
                QTimer m_timer;

                std::unique_ptr<PositionWidget> m_position_widget;
                geometry::Rectangle m_bounding_box;
                std::unique_ptr< navigation::Pose > m_start;
                std::unique_ptr< navigation::Pose > m_goal;
                std::vector< std::shared_ptr< navigation::Obstacle > > m_obstacles;
                std::vector< navigation::TrajectoryPoint > m_trajectory;
                std::vector< geometry::Point > m_waypoints;
                std::shared_ptr< navigation::RobotFootprintModel > m_robot;
            };
        }
    }
}

#endif // PATH_WIDGET_HPP
