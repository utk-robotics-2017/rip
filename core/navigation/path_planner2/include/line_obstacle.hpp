#ifndef LINE_OBSTACLE_HPP
#define LINE_OBSTACLE_HPP

namespace rip
{
    namespace navigation
    {
        /**
         * Implements a 2D line obstacle
         */
        class LineObstacle : public ObstacleBase
        {
        public:

            virtual geometry::Point centroid() const override
            {
                return m_start + (m_end - m_start) / 2.0;
            }

            virtual bool collision(const geometry::Point& point, const units::Distance& minimum_distance = 0.0) const override
            {
                return minimumDistance(point) < minimum_distance;
            }

            virtual bool lineIntersection(const geometry::Point& start, const geometry::Point& end, const units::Distance& minimum_distance = 0.0) const override
            {
                return geometry::lineIntersection(start, end, m_start, m_end);
            }

            virtual units::Distance minimumDistance(const geometry::Point& point) const override
            {
                return geometry::pointToSegment(point, m_start, m_end);
            }

            virtual units::Distance minimumDistance(const geometry::Point& start, const geometry::Point& end) const override
            {
                return geometry::segmentToSegment(start, end, m_start, m_end);
            }

            virtual units::Distance minimumDistance(const geometry::Polygon& polygon) const override
            {
                return segmentToPolygon(m_start, m_end, polygon);
            }

            virtual geometry::Point closestPoint(const geometry::Point& point) const override
            {
                return geometry::closestPointSegment(point, m_start, m_end);
            }

        private:
            Point m_start;
            Point m_end;
        };
    }
}

#endif // LINE_OBSTACLE_HPP