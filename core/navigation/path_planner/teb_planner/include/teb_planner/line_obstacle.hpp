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
            LineObstacle(const units::Distance& start_x, const units::Distance& start_y, const units::Distance& end_x, const units::Distance& end_y);

            /**
             * Constructor
             */
            LineObstacle(const geometry::Point& start, const geometry::Point& end);

            /**
             * Returns the starting point for this line segment
             */
            geometry::Point start() const;

            /**
             * Sets the start point for this line segment
             */
            void setStart(const geometry::Point& start);

            /**
             * Returns the end point for this line segment
             */
            geometry::Point end() const;

            /**
             * Sets the end point for this line segment
             */
            void setEnd(const geometry::Point& end);

            /**
             * Returns the centroid of the obstacle
             */
            virtual geometry::Point centroid() const override;

            /**
             * Moves the centroid
             */
            void setCentroid(const geometry::Point& centroid);

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
