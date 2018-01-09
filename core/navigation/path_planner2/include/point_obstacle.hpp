#ifndef POINT_OBSTACLE_HPP
#define POINT_OBSTACLE_HPP

namespace rip
{
    namespace navigation
    {
        /**
         * Implements a 2D point obstacle
         *
         * @ref https://github.com/rst-tu-dortmund/teb_local_planner
         */
        class PointObstacle : public ObstacleBase
        {
        public:
            PointObstacle() = default;

            PointObstacle(const Point& point)
                : m_point(point)
            {}

            virtual bool collision(const Point& point, const units::Distance& minimum_distance = 0.0) const override
            {
                return point == m_point || minimumDistance(point) < minimum_distance;
            }

            virtual bool lineIntersection(const Point& start, const Point& end, const units::Distance& minimum_distance = 0.0) const override
            {
                // Distance Line - Circle
                // refer to http://www.spieleprogrammierer.de/wiki/2D-Kollisionserkennung#Kollision_Kreis-Strecke
                Point a = end - start;
                Point b = m_point - start;

                // Now find nearest point to circle v=x+a*t with t=a*b/(a*a) and bound to 0<=t<=1
                double t = a.dot(b)() / a.dot(a)();

                // t is bound since a isn't normalized
                if (t < 0)
                {
                    t = 0;
                }
                else if (t > 1)
                {
                    t = 1;
                }
                Point nearest = start + a * t;

                return collision(nearest, minimum_distance);
            }

            virtual units::Distance minimumDistance(const Point& point) const override
            {
                return m_point.distance(point);
            }

            virtual units::Distance minimumDistance(const Point& start, const Point& end) const override
            {
                return pointToSegment(m_point, start, end);
            }

            virtual units::Distance minimumDistance(const Polygon& polygon)
            {
                return pointToPolygon(m_point, polygon);
            }

            virtual Point closestPoint(const Point& point) const
            {
                return m_point;
            }

            virtual Point centroid() const
            {
                return m_point;
            }
        protected:
            geometry::Point m_point;
        };
    }
}

#endif // POINT_OBSTACLE_HPP