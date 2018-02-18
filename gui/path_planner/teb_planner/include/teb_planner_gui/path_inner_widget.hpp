#ifndef PATH_INNER_WIDGET_HPP
#define PATH_INNER_WIDGET_HPP

#include <memory>
#include <vector>

#include <QWidget>
#include <QPainter>
#include <QPaintEvent>
#include <QResizeEvent>
#include <QTimer>

#include <teb_planner/robot_footprint_model.hpp>
#include <teb_planner/obstacles.hpp>
#include <teb_planner/trajectory_point.hpp>
#include <teb_planner/pose.hpp>

#include "position_widget.hpp"
#include "angle_widget.hpp"
#include "compute_thread.hpp"

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
            class PathInnerWidget : public QWidget
            {
                Q_OBJECT
            public:
                /**
                 * Constructor
                 */
                explicit PathInnerWidget(QWidget* parent = nullptr);

                /**
                 * Sets whether the robot should be animated
                 */
                void setAnimate(bool animate);

                void setStart(const navigation::tebplanner::Pose& start);

                void setGoal(const navigation::tebplanner::Pose& goal);

                /**
                 * Set the robot footprint model
                 */
                void setRobot(std::shared_ptr< navigation::tebplanner::BaseRobotFootprintModel > robot);

                /**
                 * Sets the obstacles
                 */
                void setObstacles(std::shared_ptr< std::vector< std::shared_ptr<navigation::tebplanner::Obstacle > > > obstacles);

            private slots:
                void updateSelectedPosition();

                void addPointObstacle();

                void addLineObstacle();

                void addPolygonObstacle();

                void trajectoryUpdated();

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

                /**
                 * Handle mouse drag events
                 */
                virtual void mouseMoveEvent(QMouseEvent* event) override;

                /**
                 * Handle mosue release events
                 */
                virtual void mouseReleaseEvent(QMouseEvent* event) override;
            private:
                QMatrix getTransform(double* scale = nullptr) const;

                /**
                 * Draw the bounding box
                 */
                void drawBoundingBox(QPainter& painter, double scale);

                /**
                 * Draw the start and goal poses
                 */
                void drawStartAndGoal(QPainter& painter, double scale);

                void drawPose(const navigation::tebplanner::Pose& pose, Qt::GlobalColor color, QPainter& painter, double scale);

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
                    kNone,
                    kStartPoint,
                    kStartDirection,
                    kGoalPoint,
                    kGoalDirection,
                    kWaypoint,
                    kPoint,
                    kLineStart,
                    kLineEnd,
                    kLine,
                    kPolygonPoint,
                    kPolygon
                };

                bool checkPointClick(const geometry::Point& point, const QMatrix& transform, const QPoint& mouse_pos, SelectedType select, int index = -1, int polygon_index = -1, unsigned int pixel_theshold = 15);


                SelectedType m_selected_type;
                int m_selected_index;
                int m_selected_polygon_index;
                std::unique_ptr<PositionWidget> m_position_widget;
                std::unique_ptr<AngleWidget> m_angle_widget;

                bool m_animate;
                int m_timestep;
                QTimer m_timer;


                geometry::Rectangle m_bounding_box;
                std::shared_ptr< navigation::tebplanner::Pose > m_start;
                std::shared_ptr< navigation::tebplanner::Pose > m_goal;
                std::shared_ptr< std::vector< std::shared_ptr< navigation::tebplanner::Obstacle > > > m_obstacles;
                std::shared_ptr< navigation::tebplanner::BaseRobotFootprintModel > m_robot;
                std::vector< navigation::tebplanner::TrajectoryPoint > m_trajectory;
                std::vector< geometry::Point > m_waypoints;

                std::shared_ptr< ComputeThread > m_compute_thread;
            };
        }
    }
}

#endif // PATH_INNER_WIDGET_HPP
