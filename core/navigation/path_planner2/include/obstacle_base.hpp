#ifndef OBSTACLE_BASE_HPP
#define OBSTACLE_BASE_HPP

namespace rip
{
    namespace navigation
    {
        /**
         * Abstract base that defines the interface for modeling obstacles
         */
        class ObstacleBase
        {
        public:
            /**
             * Default Constructor
             */
            ObstacleBase()
                : m_dynamic(false)
            {}

            /**
             * Returns the centroid of the obstacle
             */
            virtual geometry::Point centroid() const  = 0;

            /**
             * Returns if the given point intersects with
             * the obstacle or is with in the given distance
             */
            virtual bool collision(const geometry::Point& point, const units::Distance& minimum_distance = 0.0) const = 0;

            /**
             * Returns if the given line intersects with
             * the obstacle or is with in the given distance
             */
            virtual bool lineIntersection(const geometry::Point& start, const geometry::Point& end, const units::Distance& minimum_distance = 0.0) const = 0;

            /**
             *
             * Returns the minimum distance from the point to the obstacle
             */
            virtual units::Distance minimumDistance(const geometry::Point& point) const = 0;

            /**
             * Returns the minimum distance from the line to the obstacle
             */
            virtual units::Distance minimumDistance(const geometry::Point& start, const geometry::Point& end) = 0;

            /**
             * Returns the minimum distance from the polygon to the obstacle
             */
            virtual units::Distance minimumDistance(const geometry::Polygon& polygon) const = 0;

            /**
             * Returns the closest point on the boundary of the obstacle to the
             * specified point
             */
            virtual geometry::Point closestPoint(const geometry::Point& point) const = 0;

            // todo moving obstacles
        protected:
            bool m_dynamic;
            geometry::Point m_centroid_velocity;
        };
    }
}

#endif // OBSTACLE_BASE_HPP