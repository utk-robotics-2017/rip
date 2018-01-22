#ifndef OBSTACLE_BASE_HPP
#define OBSTACLE_BASE_HPP

#include <units/units.hpp>
#include <geometry/point.hpp>
#include <geometry/polygon.hpp>

#include "velocity_pose.hpp"

namespace rip
{
    namespace navigation
    {
        /**
         * Abstract base that defines the interface for modeling obstacles
         */
        class Obstacle
        {
        public:
            /**
             * Default Constructor
             */
            Obstacle();

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
             * Returns the minimum distance from the point to the obstacle
             */
            virtual units::Distance minimumDistance(const geometry::Point& point) const = 0;

            /**
             * Returns the minimum distance from the line to the obstacle
             */
            virtual units::Distance minimumDistance(const geometry::Point& start, const geometry::Point& end) const = 0;

            /**
             * Returns the minimum distance from the polygon to the obstacle
             */
            virtual units::Distance minimumDistance(const geometry::Polygon& polygon) const = 0;

            /**
             * Returns the closest point on the boundary of the obstacle to the
             * specified point
             */
            virtual geometry::Point closestPoint(const geometry::Point& point) const = 0;

            /**
             * Get the estimated distance to the moving obstacle using a constant velocity model
             */
            virtual units::Distance minimumSpatioTemporalDistance(const geometry::Point& point, const units::Time& t) const = 0;

            /**
             * Get the estimated distance to the moving obstacle using a constant velocity model
             */
            virtual units::Distance minimumSpatioTemporalDistance(const geometry::Point& start, const geometry::Point& end, const units::Time& t) const = 0;

            /**
             * Get the estimated distance to the moving obstacle using a constant velocity model
             */
            virtual units::Distance minimumSpatioTemporalDistance(const geometry::Polygon& polygon, const units::Time& t) const = 0;

            bool isDynamic() const;

            void setCentoidVelocity(const VelocityPose& velocity);

            VelocityPose getCentroidVelocity() const;

        protected:
            bool m_dynamic;
            VelocityPose m_velocity;
        };
    }
}

#endif // OBSTACLE_BASE_HPP
