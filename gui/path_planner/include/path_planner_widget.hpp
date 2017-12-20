#ifndef PATH_PLANNER_WIDGET_HPP
#define PATH_PLANNER_WIDGET_HPP

#include <memory>

#include <QWidget>

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
            public:
                explicit PathPlannerWidget(QWidget* parent = nullptr);

                void setWorld(const geometry::PolygonList& world);

            protected:
                void mouseReleaseEvent(QMouseEvent* event) override;
                void paintEvent(QPaintEvent* event) override;
                void resizeEvent(QResizeEvent *event) override;

            private:
                void drawWorld(QPainter& painter);
                void drawWaypoints(QPainter& painter);
                void drawTrajectory(QPainter& painter);
                void showContextMenu(const QPoint& point);

                geometry::PolygonList m_world;
                std::shared_ptr<ComputeThread> m_compute_thread;
            }; // class PathPlannerWidget
        } // namespace pathplanner
    } // namespace gui
} // namespace rip

#endif // PATH_PLANNER_WIDGET_HPP
