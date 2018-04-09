#ifndef WORLD_WIDGET_INNER_HPP
#define WORLD_WIDGET_INNER_HPP

#include <memory>

#include <QWidget>

#include "path_follower_gui/world.hpp"
#include "path_follower_gui/position_widget.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            namespace gui
            {
                class WorldWidgetInner : public QWidget
                {
                    Q_OBJECT
                public:
                    explicit WorldWidgetInner(QWidget* parent);

                public slots:
                    void updateWorld();

                private slots:
                    void updateSelectedPosition();

                    void addObstacle();

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

                    enum class SelectedType
                    {
                        kNone,
                        kObstacle,
                        kObstaclePoint,
                        kBoundaryPoint
                    };

                    bool checkPointClick(const geometry::Point& point, const QMatrix& transform, const QPoint& mouse_pos, SelectedType select, int index = -1, int polygon_index = -1, unsigned int pixel_theshold = 15);

                    /**
                     * Show the context menu when right clicked
                     */
                    void showContextMenu(const QPoint& point);

                    std::shared_ptr<World> m_world;
                    geometry::Rectangle m_bounding_box;
                    SelectedType m_selected_type;
                    int m_selected_index;
                    int m_selected_polygon_index;
                    std::unique_ptr<PositionWidget> m_position_widget;
                };
            }
        }
    }
}

#endif // WORLD_WIDGET_INNER_HPP
