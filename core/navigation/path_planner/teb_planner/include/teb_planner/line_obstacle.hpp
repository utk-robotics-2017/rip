#ifndef LINE_OBSTACLE_HPP
#define LINE_OBSTACLE_HPP

#include "obstacle.hpp"

namespace rip
{
    namespace navigation
    {
        /**
         * Implements a 2D line obstacle
         */
        class LineObstacle : public Obstacle
        {
        public:
            /**
             * Constructor
             */
            LineObstacle(const geometry::Point& start, const geometry::Point& end);

            geometry::Point start() const;

            geometry::Point end() const;

            /**
             * Returns the centroid of the obstacle
             */
            virtual geometry::Point centroid() const override;

            /**
             * Returns if the given point intersects with
             * the obstacle or is with in the given distance
             */
            virtual bool collision(const geometry::Point& point, const units::Distance& minimum_distance = 0.0) const override;

            /**
             * Returns if the given line intersects with
             * the obstacle or is with in the given distance
             */
            virtual bool lineIntersection(const geometry::Point& start, const geometry::Point& end, const units::Distance& minimum_distance = 0.0) const override;

            /**
             * Returns the minimum distance from the point to the obstacle
             */
            virtual units::Distance minimumDistance(const geometry::Point& point) const override;

            /**
             * Returns the minimum distance from the line to the obstacle
             */
            virtual units::Distance minimumDistance(const geometry::Point& start, const geometry::Point& end) const override;

            /**
             * Returns the minimum distance from the polygon to the obstacle
             */
            virtual units::Distance minimumDistance(const geometry::Polygon& polygon) const override;

            /**
             * Returns the closest point on the boundary of the obstacle to the
             * specified point
             */
            virtual geometry::Point closestPoint(const geometry::Point& point) const override;

            /**
             * Get the estimated distance to the moving obstacle using a constant velocity model
             */
            virtual units::Distance minimumSpatioTemporalDistance(const geometry::Point& point, const units::Time& t) const override;

            /**
             * Get the estimated distance to the moving obstacle using a constant velocity model
             */
            virtual units::Distance minimumSpatioTemporalDistance(const geometry::Point& start, const geometry::Point& end, const units::Time& t) const override;

            /**
             * Get the estimated distance to the moving obstacle using a constant velocity model
             */
            virtual units::Distance minimumSpatioTemporalDistance(const geometry::Polygon& polygon, const units::Time& t) const override;
        private:
            geometry::Point m_start;
            geometry::Point m_end;
        };
    }
}

#endif // LINE_OBSTACLE_HPP
