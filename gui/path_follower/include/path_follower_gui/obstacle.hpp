#ifndef OBSTACLE_HPP
#define OBSTACLE_HPP

#include <QPainter>

#include <units/units.hpp>
#include <geometry/point.hpp>
#include <geometry/polygon.hpp>

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            namespace  gui
            {
                class Obstacle
                {
                public:
                    Obstacle(const geometry::Polygon& polygon);
                    void draw(QPainter& painter);
                    geometry::Polygon polygon() const;
                    geometry::Point center() const;
                    void setCenter(const geometry::Point& center);
                    void setPoint(int index, const geometry::Point& point);

                private:
                    geometry::Polygon m_polygon;
                };
            }
        }
    }
}

#endif // OBSTACLE_HPP
