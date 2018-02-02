/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann
 *
 * Modified by: Andrew Messing
 * - Removed all aspects that required ROS or boost and added in RIP
 *   elements
 *********************************************************************/


#ifndef OBSTACLES_HPP
#define OBSTACLES_HPP

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/StdVector>
#include <eigen3/Eigen/Geometry>

#include <complex>
#include <memory>

#include <teb_planner/distance_calculations.hpp>
#include <teb_planner/fake_ros_msgs.hpp>

// RIP
#include <geometry/point.hpp>
#include <geometry/polygon.hpp>

namespace rip
{
    namespace navigation
    {
        namespace tebplanner
        {
            /**
             * @class Obstacle
             * @brief Abstract class that defines the interface for modelling obstacles
             */
            class Obstacle
            {
            public:

                /**
                  * @brief Default constructor of the abstract obstacle class
                  */
                Obstacle()
                    : m_dynamic(false)
                    , m_centroid_velocity(Eigen::Vector2d::Zero())
                {
                }

                /**
                 * @brief Virtual destructor.
                 */
                virtual ~Obstacle()
                {
                }


                /** @name Centroid coordinates (abstract, obstacle type depending) */
                //@{

                /**
                  * @brief Get centroid coordinates of the obstacle
                  * @return Eigen::Vector2d containing the centroid
                  */
                virtual const Eigen::Vector2d& getCentroid() const = 0;

                /**
                  * @brief Get centroid coordinates of the obstacle as complex number
                  * @return std::complex containing the centroid coordinate
                  */
                virtual std::complex<double> getCentroidCplx() const = 0;

                //@}


                /** @name Collision checking and distance calculations (abstract, obstacle type depending) */
                //@{

                /**
                  * @brief Check if a given point collides with the obstacle
                  * @param position 2D reference position that should be checked
                  * @param min_dist Minimum distance allowed to the obstacle to be collision free
                  * @return \c true if position is inside the region of the obstacle or if the minimum distance is lower than min_dist
                  */
                virtual bool checkCollision(const Eigen::Vector2d& position, double min_dist) const = 0;

                /**
                  * @brief Check if a given line segment between two points intersects with the obstacle (and additionally keeps a safty distance \c min_dist)
                  * @param line_start 2D point for the end of the reference line
                  * @param line_end 2D point for the end of the reference line
                  * @param min_dist Minimum distance allowed to the obstacle to be collision/intersection free
                  * @return \c true if given line intersects the region of the obstacle or if the minimum distance is lower than min_dist
                  */
                virtual bool checkLineIntersection(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, double min_dist = 0) const = 0;

                /**
                  * @brief Get the minimum euclidean distance to the obstacle (point as reference)
                  * @param position 2d reference position
                  * @return The nearest possible distance to the obstacle
                  */
                virtual double getMinimumDistance(const Eigen::Vector2d& position) const = 0;

                /**
                 * @brief Get the minimum euclidean distance to the obstacle (line as reference)
                 * @param line_start 2d position of the begin of the reference line
                 * @param line_end 2d position of the end of the reference line
                 * @return The nearest possible distance to the obstacle
                 */
                virtual double getMinimumDistance(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end) const = 0;

                /**
                 * @brief Get the minimum euclidean distance to the obstacle (polygon as reference)
                 * @param polygon Vertices (2D points) describing a closed polygon
                 * @return The nearest possible distance to the obstacle
                 */
                virtual double getMinimumDistance(const Point2dContainer& polygon) const = 0;

                /**
                 * @brief Get the closest point on the boundary of the obstacle w.r.t. a specified reference position
                 * @param position reference 2d position
                 * @return closest point on the obstacle boundary
                 */
                virtual Eigen::Vector2d getClosestPoint(const Eigen::Vector2d& position) const = 0;

                //@}



                /** @name Velocity related methods for non-static, moving obstacles */
                //@{

                /**
                  * @brief Get the estimated minimum spatiotemporal distance to the moving obstacle using a constant velocity model (point as reference)
                  * @param position 2d reference position
                  * @param t time, for which the minimum distance to the obstacle is estimated
                  * @return The nearest possible distance to the obstacle at time t
                  */
                virtual double getMinimumSpatioTemporalDistance(const Eigen::Vector2d& position, double t) const = 0;

                /**
                  * @brief Get the estimated minimum spatiotemporal distance to the moving obstacle using a constant velocity model (line as reference)
                  * @param line_start 2d position of the begin of the reference line
                  * @param line_end 2d position of the end of the reference line
                  * @param t time, for which the minimum distance to the obstacle is estimated
                  * @return The nearest possible distance to the obstacle at time t
                  */
                virtual double getMinimumSpatioTemporalDistance(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, double t) const = 0;

                /**
                  * @brief Get the estimated minimum spatiotemporal distance to the moving obstacle using a constant velocity model (polygon as reference)
                  * @param polygon Vertices (2D points) describing a closed polygon
                  * @param t time, for which the minimum distance to the obstacle is estimated
                  * @return The nearest possible distance to the obstacle at time t
                  */
                virtual double getMinimumSpatioTemporalDistance(const Point2dContainer& polygon, double t) const = 0;

                /**
                  * @brief Predict position of the centroid assuming a constant velocity model
                  * @param[in]  t         time in seconds for the prediction (t>=0)
                  * @param[out] position  predicted 2d position of the centroid
                  */
                virtual void predictCentroidConstantVelocity(double t, Eigen::Ref<Eigen::Vector2d> position) const
                {
                    position = getCentroid() + t * getCentroidVelocity();
                }

                /**
                  * @brief Check if the obstacle is a moving with a (non-zero) velocity
                  * @return \c true if the obstacle is not marked as static, \c false otherwise
                  */
                bool isDynamic() const
                {
                    return m_dynamic;
                }

                /**
                  * @brief Set the 2d velocity (vx, vy) of the obstacle w.r.t to the centroid
                  * @remarks Setting the velocity using this function marks the obstacle as dynamic (@see isDynamic)
                  * @param vel 2D vector containing the velocities of the centroid in x and y directions
                  */
                void setCentroidVelocity(const Eigen::Ref<const Eigen::Vector2d>& vel)
                {
                    m_centroid_velocity = vel;
                    m_dynamic = true;
                }

                /**
                  * @brief Set the 2d velocity (vx, vy) of the obstacle w.r.t to the centroid
                  * @remarks Setting the velocity using this function marks the obstacle as dynamic (@see isDynamic)
                  * @param velocity fakeros::TwistWithCovariance containing the velocity of the obstacle
                  * @param orientation fakeros::QuaternionStamped containing the orientation of the obstacle
                  */
                void setCentroidVelocity(const fakeros::TwistWithCovariance& velocity,
                                         const fakeros::Quaternion& orientation)
                {
                    // Set velocity, if obstacle is moving
                    Eigen::Vector2d vel;
                    vel.coeffRef(0) = velocity.twist.linear.x;
                    vel.coeffRef(1) = velocity.twist.linear.y;

                    // If norm of velocity is less than 0.001, consider obstacle as not dynamic
                    // TODO: Get rid of constant
                    if (vel.norm() < 0.001)
                    {
                        return;
                    }

                    setCentroidVelocity(vel);
                }

                void setCentroidVelocity(const fakeros::TwistWithCovariance& velocity,
                                         const fakeros::QuaternionStamped& orientation)
                {
                    setCentroidVelocity(velocity, orientation.quaternion);
                }

                /**
                  * @brief Get the obstacle velocity (vx, vy) (w.r.t. to the centroid)
                  * @returns 2D vector containing the velocities of the centroid in x and y directions
                  */
                const Eigen::Vector2d& getCentroidVelocity() const {return m_centroid_velocity;}

                //@}

            protected:
                bool m_dynamic; //!< Store flag if obstacle is dynamic (resp. a moving obstacle)
                Eigen::Vector2d m_centroid_velocity; //!< Store the corresponding velocity (vx, vy) of the centroid (zero, if _dynamic is \c true)

            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            };


            //! Abbrev. for shared obstacle pointers
            typedef std::shared_ptr<Obstacle> ObstaclePtr;

            //! Abbrev. for containers storing multiple obstacles
            typedef std::vector<ObstaclePtr> ObstacleContainer;



            /**
             * @class PointObstacle
             * @brief Implements a 2D point obstacle
             */
            class PointObstacle : public Obstacle
            {
            public:

                /**
                  * @brief Default constructor of the point obstacle class
                  */
                PointObstacle()
                    : Obstacle()
                    , m_pos(Eigen::Vector2d::Zero())
                {}

                /**
                  * @brief Construct PointObstacle using a 2d position vector
                  * @param position 2d position that defines the current obstacle position
                  */
                PointObstacle(const Eigen::Ref< const Eigen::Vector2d>& position)
                    : Obstacle()
                    , m_pos(position)
                {}

                /**
                  * @brief Construct PointObstacle using x- and y-coordinates
                  * @param x x-coordinate
                  * @param y y-coordinate
                  */
                PointObstacle(double x, double y)
                    : Obstacle()
                    , m_pos(Eigen::Vector2d(x, y))
                {}

                PointObstacle(const geometry::Point& p)
                    : Obstacle()
                    , m_pos(Eigen::Vector2d(p.x().to(units::m), p.y().to(units::m)))
                {}


                // implements checkCollision() of the base class
                virtual bool checkCollision(const Eigen::Vector2d& point, double min_dist) const
                {
                    return getMinimumDistance(point) < min_dist;
                }


                // implements checkLineIntersection() of the base class
                virtual bool checkLineIntersection(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, double min_dist = 0) const
                {
                    // Distance Line - Circle
                    // refer to http://www.spieleprogrammierer.de/wiki/2D-Kollisionserkennung#Kollision_Kreis-Strecke
                    Eigen::Vector2d a = line_end - line_start; // not normalized!  a=y-x
                    Eigen::Vector2d b = m_pos - line_start; // b=m-x

                    // Now find nearest point to circle v=x+a*t with t=a*b/(a*a) and bound to 0<=t<=1
                    double t = a.dot(b) / a.dot(a);
                    if (t < 0) { t = 0; } // bound t (since a is not normalized, t can be scaled between 0 and 1 to parametrize the line
                    else if (t > 1) { t = 1; }
                    Eigen::Vector2d nearest_point = line_start + a * t;

                    // check collision
                    return checkCollision(nearest_point, min_dist);
                }


                // implements getMinimumDistance() of the base class
                virtual double getMinimumDistance(const Eigen::Vector2d& position) const
                {
                    return (position - m_pos).norm();
                }

                // implements getMinimumDistance() of the base class
                virtual double getMinimumDistance(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end) const
                {
                    return distance_point_to_segment_2d(m_pos, line_start, line_end);
                }

                // implements getMinimumDistance() of the base class
                virtual double getMinimumDistance(const Point2dContainer& polygon) const
                {
                    return distance_point_to_polygon_2d(m_pos, polygon);
                }

                // implements getMinimumDistanceVec() of the base class
                virtual Eigen::Vector2d getClosestPoint(const Eigen::Vector2d& position) const
                {
                    return m_pos;
                }

                // implements getMinimumSpatioTemporalDistance() of the base class
                virtual double getMinimumSpatioTemporalDistance(const Eigen::Vector2d& position, double t) const
                {
                    return (m_pos + t * m_centroid_velocity - position).norm();
                }

                // implements getMinimumSpatioTemporalDistance() of the base class
                virtual double getMinimumSpatioTemporalDistance(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, double t) const
                {
                    return distance_point_to_segment_2d(m_pos + t * m_centroid_velocity, line_start, line_end);
                }

                // implements getMinimumSpatioTemporalDistance() of the base class
                virtual double getMinimumSpatioTemporalDistance(const Point2dContainer& polygon, double t) const
                {
                    return distance_point_to_polygon_2d(m_pos + t * m_centroid_velocity, polygon);
                }

                // implements predictCentroidConstantVelocity() of the base class
                virtual void predictCentroidConstantVelocity(double t, Eigen::Ref<Eigen::Vector2d> position) const
                {
                    position = m_pos + t * m_centroid_velocity;
                }

                geometry::Point getCentroidPoint() const
                {
                    return geometry::Point(x() * units::m, y() * units::m);
                }

                // implements getCentroid() of the base class
                virtual const Eigen::Vector2d& getCentroid() const
                {
                    return m_pos;
                }

                // implements getCentroidCplx() of the base class
                virtual std::complex<double> getCentroidCplx() const
                {
                    return std::complex<double>(m_pos[0], m_pos[1]);
                }

                void setPosition(const geometry::Point& p)
                {
                    x() = p.x().to(units::m);
                    y() = p.y().to(units::m);
                }

                geometry::Point point() const
                {
                    return geometry::Point(x() * units::m, y() * units::m);
                }

                // Accessor methods
                const Eigen::Vector2d& position() const
                {
                    return m_pos;
                } //!< Return the current position of the obstacle (read-only)

                Eigen::Vector2d& position()
                {
                    return m_pos;
                } //!< Return the current position of the obstacle

                double& x()
                {
                    return m_pos.coeffRef(0);
                } //!< Return the current x-coordinate of the obstacle

                const double& x() const
                {
                    return m_pos.coeffRef(0);
                } //!< Return the current y-coordinate of the obstacle (read-only)

                double& y()
                {
                    return m_pos.coeffRef(1);
                } //!< Return the current x-coordinate of the obstacle

                const double& y() const
                {
                    return m_pos.coeffRef(1);
                } //!< Return the current y-coordinate of the obstacle (read-only)

            protected:
                Eigen::Vector2d m_pos; //!< Store the position of the PointObstacle

            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            };



            /**
            * @class LineObstacle
            * @brief Implements a 2D line obstacle
            */
            class LineObstacle : public Obstacle
            {
            public:
                //! Abbrev. for a container storing vertices (2d points defining the edge points of the polygon)
                typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > VertexContainer;

                /**
                  * @brief Default constructor of the point obstacle class
                  */
                LineObstacle()
                    : Obstacle()
                {
                    m_start.setZero();
                    m_end.setZero();
                    m_centroid.setZero();
                }

                /**
                 * @brief Construct LineObstacle using 2d position vectors as start and end of the line
                 * @param line_start 2d position that defines the start of the line obstacle
                 * @param line_end 2d position that defines the end of the line obstacle
                 */
                LineObstacle(const Eigen::Ref< const Eigen::Vector2d>& line_start, const Eigen::Ref< const Eigen::Vector2d>& line_end)
                    : Obstacle(), m_start(line_start), m_end(line_end)
                {
                    calcCentroid();
                }

                /**
                 * @brief Construct LineObstacle using start and end coordinates
                 * @param x1 x-coordinate of the start of the line
                 * @param y1 y-coordinate of the start of the line
                 * @param x2 x-coordinate of the end of the line
                 * @param y2 y-coordinate of the end of the line
                 */
                LineObstacle(double x1, double y1, double x2, double y2)
                    : Obstacle()
                {
                    m_start.x() = x1;
                    m_start.y() = y1;
                    m_end.x() = x2;
                    m_end.y() = y2;
                    calcCentroid();
                }

                LineObstacle(const geometry::Point& start, const geometry::Point& end)
                    : Obstacle()
                    , m_start(start.x().to(units::m), start.y().to(units::m))
                    , m_end(end.x().to(units::m), end.y().to(units::m))
                {
                    calcCentroid();
                }

                // implements checkCollision() of the base class
                virtual bool checkCollision(const Eigen::Vector2d& point, double min_dist) const
                {
                    return getMinimumDistance(point) <= min_dist;
                }

                // implements checkLineIntersection() of the base class
                virtual bool checkLineIntersection(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, double min_dist = 0) const
                {
                    return check_line_segments_intersection_2d(line_start, line_end, m_start, m_end);
                }

                // implements getMinimumDistance() of the base class
                virtual double getMinimumDistance(const Eigen::Vector2d& position) const
                {
                    return distance_point_to_segment_2d(position, m_start, m_end);
                }

                // implements getMinimumDistance() of the base class
                virtual double getMinimumDistance(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end) const
                {
                    return distance_segment_to_segment_2d(m_start, m_end, line_start, line_end);
                }

                // implements getMinimumDistance() of the base class
                virtual double getMinimumDistance(const Point2dContainer& polygon) const
                {
                    return distance_segment_to_polygon_2d(m_start, m_end, polygon);
                }

                // implements getMinimumDistanceVec() of the base class
                virtual Eigen::Vector2d getClosestPoint(const Eigen::Vector2d& position) const
                {
                    return closest_point_on_line_segment_2d(position, m_start, m_end);
                }

                // implements getMinimumSpatioTemporalDistance() of the base class
                virtual double getMinimumSpatioTemporalDistance(const Eigen::Vector2d& position, double t) const
                {
                    Eigen::Vector2d offset = t * m_centroid_velocity;
                    return distance_point_to_segment_2d(position, m_start + offset, m_end + offset);
                }

                // implements getMinimumSpatioTemporalDistance() of the base class
                virtual double getMinimumSpatioTemporalDistance(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, double t) const
                {
                    Eigen::Vector2d offset = t * m_centroid_velocity;
                    return distance_segment_to_segment_2d(m_start + offset, m_end + offset, line_start, line_end);
                }

                // implements getMinimumSpatioTemporalDistance() of the base class
                virtual double getMinimumSpatioTemporalDistance(const Point2dContainer& polygon, double t) const
                {
                    Eigen::Vector2d offset = t * m_centroid_velocity;
                    return distance_segment_to_polygon_2d(m_start + offset, m_end + offset, polygon);
                }

                // implements getCentroid() of the base class
                virtual const Eigen::Vector2d& getCentroid() const
                {
                    return m_centroid;
                }

                geometry::Point getCentroidPoint() const
                {
                    return geometry::Point(m_centroid.coeffRef(0) * units::m, m_centroid.coeffRef(1) * units::m);
                }

                void setCentroid(const geometry::Point& p)
                {
                    geometry::Point diff = p - getCentroidPoint();
                    m_start += Eigen::Vector2d(diff.x().to(units::m), diff.y().to(units::m));
                    m_end += Eigen::Vector2d(diff.x().to(units::m), diff.y().to(units::m));
                    calcCentroid();
                }

                // implements getCentroidCplx() of the base class
                virtual std::complex<double> getCentroidCplx() const
                {
                    return std::complex<double>(m_centroid.x(), m_centroid.y());
                }

                geometry::Point startPoint() const
                {
                    return geometry::Point(m_start.x() * units::m, m_start.y() * units::m);
                }

                void setStart(const geometry::Point& p)
                {
                    m_start = Eigen::Vector2d(p.x().to(units::m), p.y().to(units::m));
                    calcCentroid();
                }

                // Access or modify line
                const Eigen::Vector2d& start() const
                {
                    return m_start;
                }

                void setStart(const Eigen::Ref<const Eigen::Vector2d>& start)
                {
                    m_start = start;
                    calcCentroid();
                }

                geometry::Point endPoint() const
                {
                    return geometry::Point(m_end.x() * units::m, m_end.y() * units::m);
                }

                void setEnd(const geometry::Point& p)
                {
                    m_end = Eigen::Vector2d(p.x().to(units::m), p.y().to(units::m));
                    calcCentroid();
                }

                const Eigen::Vector2d& end() const
                {
                    return m_end;
                }

                void setEnd(const Eigen::Ref<const Eigen::Vector2d>& end)
                {
                    m_end = end;
                    calcCentroid();
                }

            protected:
                void calcCentroid()
                {
                    m_centroid = 0.5 * (m_start + m_end);
                }

            private:
                Eigen::Vector2d m_start;
                Eigen::Vector2d m_end;

                Eigen::Vector2d m_centroid;

            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            };


            /**
             * @class PolygonObstacle
             * @brief Implements a polygon obstacle with an arbitrary number of vertices
             * @details If the polygon has only 2 vertices, than it is considered as a line,
             *      otherwise the polygon will always be closed (a connection between the first and the last vertex
             *      is included automatically).
             */
            class PolygonObstacle : public Obstacle
            {
            public:

                /**
                  * @brief Default constructor of the polygon obstacle class
                  */
                PolygonObstacle()
                    : Obstacle()
                    , m_finalized(false)
                {
                    m_centroid.setConstant(NAN);
                }

                /**
                 * @brief Construct polygon obstacle with a list of vertices
                 */
                PolygonObstacle(const Point2dContainer& vertices)
                    : Obstacle(), m_vertices(vertices)
                {
                    finalizePolygon();
                }

                PolygonObstacle(const geometry::Polygon& polygon)
                    : Obstacle()
                {
                    setPolygon(polygon);
                }


                // implements checkCollision() of the base class
                virtual bool checkCollision(const Eigen::Vector2d& point, double min_dist) const
                {
                    // line case
                    if (noVertices() == 2)
                    {
                        return getMinimumDistance(point) <= min_dist;
                    }

                    // check if point is in the interior of the polygon
                    // point in polygon test - raycasting (http://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html)
                    // using the following algorithm we may obtain false negatives on edge-cases, but that's ok for our purposes
                    int i, j;
                    bool c = false;
                    for (i = 0, j = noVertices() - 1; i < noVertices(); j = i++)
                    {
                        if ( ((m_vertices.at(i).y() > point.y()) != (m_vertices.at(j).y() > point.y())) &&
                                (point.x() < (m_vertices.at(j).x() - m_vertices.at(i).x()) * (point.y() - m_vertices.at(i).y()) / (m_vertices.at(j).y() - m_vertices.at(i).y()) + m_vertices.at(i).x()) )
                        {
                            c = !c;
                        }
                    }
                    if (c > 0) { return true; }

                    // If this statement is reached, the point lies outside the polygon or maybe on its edges
                    // Let us check the minium distance as well
                    return min_dist == 0 ? false : getMinimumDistance(point) < min_dist;
                }


                /**
                  * @brief Check if a given line segment between two points intersects with the obstacle (and additionally keeps a safty distance \c min_dist)
                  * @param line_start 2D point for the end of the reference line
                  * @param line_end 2D point for the end of the reference line
                  * @param min_dist Minimum distance allowed to the obstacle to be collision/intersection free
                  * @remarks we ignore \c min_dist here
                  * @return \c true if given line intersects the region of the obstacle or if the minimum distance is lower than min_dist
                  */
                virtual bool checkLineIntersection(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, double min_dist = 0) const;


                // implements getMinimumDistance() of the base class
                virtual double getMinimumDistance(const Eigen::Vector2d& position) const
                {
                    return distance_point_to_polygon_2d(position, m_vertices);
                }

                // implements getMinimumDistance() of the base class
                virtual double getMinimumDistance(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end) const
                {
                    return distance_segment_to_polygon_2d(line_start, line_end, m_vertices);
                }

                // implements getMinimumDistance() of the base class
                virtual double getMinimumDistance(const Point2dContainer& polygon) const
                {
                    return distance_polygon_to_polygon_2d(polygon, m_vertices);
                }

                // implements getMinimumDistanceVec() of the base class
                virtual Eigen::Vector2d getClosestPoint(const Eigen::Vector2d& position) const;

                // implements getMinimumSpatioTemporalDistance() of the base class
                virtual double getMinimumSpatioTemporalDistance(const Eigen::Vector2d& position, double t) const
                {
                    Point2dContainer pred_vertices;
                    predictVertices(t, pred_vertices);
                    return distance_point_to_polygon_2d(position, pred_vertices);
                }

                // implements getMinimumSpatioTemporalDistance() of the base class
                virtual double getMinimumSpatioTemporalDistance(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, double t) const
                {
                    Point2dContainer pred_vertices;
                    predictVertices(t, pred_vertices);
                    return distance_segment_to_polygon_2d(line_start, line_end, pred_vertices);
                }

                // implements getMinimumSpatioTemporalDistance() of the base class
                virtual double getMinimumSpatioTemporalDistance(const Point2dContainer& polygon, double t) const
                {
                    Point2dContainer pred_vertices;
                    predictVertices(t, pred_vertices);
                    return distance_polygon_to_polygon_2d(polygon, pred_vertices);
                }

                virtual void predictVertices(double t, Point2dContainer& pred_vertices) const
                {
                    // Predict obstacle (polygon) at time t
                    pred_vertices.resize(m_vertices.size());
                    Eigen::Vector2d offset = t * m_centroid_velocity;
                    for (std::size_t i = 0; i < m_vertices.size(); i++)
                    {
                        pred_vertices[i] = m_vertices[i] + offset;
                    }
                }

                void setCentroid(const geometry::Point& p)
                {
                    geometry::Point diff = p - getCentroidPoint();
                    Eigen::Vector2d eigen_diff(diff.x().to(units::m), diff.y().to(units::m));

                    for (Eigen::Vector2d& point : m_vertices)
                    {
                        point += eigen_diff;
                    }
                    finalizePolygon();
                }

                geometry::Point getCentroidPoint() const
                {
                    return geometry::Point(m_centroid.x() * units::m, m_centroid.y() * units::m);
                }

                // implements getCentroid() of the base class
                virtual const Eigen::Vector2d& getCentroid() const
                {
                    assert(m_finalized && "Finalize the polygon after all vertices are added.");
                    return m_centroid;
                }

                // implements getCentroidCplx() of the base class
                virtual std::complex<double> getCentroidCplx() const
                {
                    assert(m_finalized && "Finalize the polygon after all vertices are added.");
                    return std::complex<double>(m_centroid.coeffRef(0), m_centroid.coeffRef(1));
                }


                /** @name Define the polygon */
                ///@{

                geometry::Polygon polygon() const
                {
                    geometry::Polygon poly;
                    for (const Eigen::Vector2d& point : m_vertices)
                    {
                        poly.add(geometry::Point(point.x() * units::m, point.y() * units::m));
                    }
                    return poly;
                }

                void setPolygon(const geometry::Polygon& polygon)
                {
                    m_vertices.clear();
                    for (const geometry::Point& point : polygon)
                    {
                        m_vertices.push_back(Eigen::Vector2d(point.x().to(units::m), point.y().to(units::m)));
                    }
                    finalizePolygon();
                }

                void setPoint(unsigned int index, const geometry::Point& p)
                {
                    assert(index < m_vertices.size());
                    m_vertices[index] = Eigen::Vector2d(p.x().to(units::m), p.y().to(units::m));
                    finalizePolygon();
                }

                // Access or modify polygon
                const Point2dContainer& vertices() const
                {
                    return m_vertices;
                } //!< Access vertices container (read-only)

                Point2dContainer& vertices()
                {
                    return m_vertices;
                } //!< Access vertices container

                /**
                  * @brief Add a vertex to the polygon (edge-point)
                  * @remarks You do not need to close the polygon (do not repeat the first vertex)
                  * @warning Do not forget to call finalizePolygon() after adding all vertices
                  * @param vertex 2D point defining a new polygon edge
                  */
                void pushBackVertex(const Eigen::Ref<const Eigen::Vector2d>& vertex)
                {
                    m_vertices.push_back(vertex);
                    m_finalized = false;
                }

                /**
                  * @brief Add a vertex to the polygon (edge-point)
                  * @remarks You do not need to close the polygon (do not repeat the first vertex)
                  * @warning Do not forget to call finalizePolygon() after adding all vertices
                  * @param x x-coordinate of the new vertex
                  * @param y y-coordinate of the new vertex
                  */
                void pushBackVertex(double x, double y)
                {
                    m_vertices.push_back(Eigen::Vector2d(x, y));
                    m_finalized = false;
                }

                /**
                  * @brief Call finalizePolygon after the polygon is created with the help of pushBackVertex() methods
                  */
                void finalizePolygon()
                {
                    fixPolygonClosure();
                    calcCentroid();
                    m_finalized = true;
                }

                /**
                  * @brief Clear all vertices (Afterwards the polygon is not valid anymore)
                  */
                void clearVertices() {m_vertices.clear(); m_finalized = false;}

                /**
                  * @brief Get the number of vertices defining the polygon (the first vertex is counted once)
                  */
                int noVertices() const {return (int)m_vertices.size();}


                ///@}

            protected:

                void fixPolygonClosure(); //!< Check if the current polygon contains the first vertex twice (as start and end) and in that case erase the last redundant one.

                void calcCentroid(); //!< Compute the centroid of the polygon (called inside finalizePolygon())


                Point2dContainer m_vertices; //!< Store vertices defining the polygon (@see pushBackVertex)
                Eigen::Vector2d m_centroid; //!< Store the centroid coordinates of the polygon (@see calcCentroid)

                bool m_finalized; //!< Flat that keeps track if the polygon was finalized after adding all vertices


            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            };


        }
    }
}

#endif /* OBSTACLES_H */
