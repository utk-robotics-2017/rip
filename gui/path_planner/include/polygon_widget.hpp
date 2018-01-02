#ifndef POLYGON_WIDGET_HPP
#define POLYGON_WIDGET_HPP

#include <QWidget>

#include <polygon.hpp>

namespace rip
{
    namespace gui
    {
        namespace pathplanner
        {
            class PolygonWidget : public QWidget
            {
                Q_OBJECT
            public:
                explicit PolygonWidget(QWidget* parent = nullptr);

            public slots:
                void updatePolygon(const geometry::Polygon& polygon);

            protected:
                virtual void paintEvent(QPaintEvent* event) override;

                virtual void resizeEvent(QResizeEvent* event) override;

            private:
                geometry::Polygon m_polygon;
            };
        }
    }
}

#endif // POLYGON_WIDGET_HPP
