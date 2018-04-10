#include "path_follower_gui/obstacle.hpp"
#include "path_follower_gui/draw_utilities.hpp"
namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            namespace  gui
            {

                Obstacle::Obstacle(const geometry::Polygon& polygon)
                    : m_polygon(polygon)
                {}

                void Obstacle::draw(QPainter& painter)
                {
                    painter.setBrush(Qt::black);
                    DrawUtilities::drawPolygon(painter, m_polygon);
                }

                geometry::Polygon Obstacle::polygon() const
                {
                    return m_polygon;
                }

                geometry::Point Obstacle::center() const
                {
                    return m_polygon.centroid();
                }

                void Obstacle::setCenter(const geometry::Point& center)
                {
                    geometry::Point diff = center - m_polygon.centroid();
                    for(geometry::Point& point : m_polygon)
                    {
                        point += diff;
                    }
                }

                void Obstacle::setPoint(int index, const geometry::Point& point)
                {
                    m_polygon[index] = point;
                }

            }
        }
    }
}
