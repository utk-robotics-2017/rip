#ifndef PATH_PLANNER_WIDGET_HPP
#define PATH_PLANNER_WIDGET_HPP

#include <memory>

#include <QWidget>
#include <QTimer>
#include <QTime>

#include <polygon_list.hpp>

#include "compute_thread.hpp"

namespace rip
{
    namespace gui
    {
        namespace pathplanner
        {
            class PathPlannerWidget : public QWidget
            {
                Q_OBJECT
            public:
                explicit PathPlannerWidget(QWidget* parent = nullptr);

                void setWorld(const geometry::Polygon& world);
                void setAnimate(bool animate);
                void setSpeedUp(float speed_up);
                void setDisplay(const QString& display);

            protected:
                void paintEvent(QPaintEvent* event) override;
                void resizeEvent(QResizeEvent *event) override;

            private:
                QPolygonF createRect(const QPointF& center, double rotation, double width, double length);

                void drawWorld(QPainter& painter);
                void drawWaypoints(QPainter& painter, double scale);
                void drawTrajectory(QPainter& painter, double scale);

                geometry::Polygon m_world;
                std::shared_ptr<ComputeThread> m_compute_thread;
                QTimer m_timer;
                QTime m_start;
                bool m_animate;
                float m_speed_up;
                QString m_display;
            }; // class PathPlannerWidget
        } // namespace pathplanner
    } // namespace gui
} // namespace rip

#endif // PATH_PLANNER_WIDGET_HPP
