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


#ifndef ROBOT_FOOTPRINT_MODEL_HPP
#define ROBOT_FOOTPRINT_MODEL_HPP

#include <teb_planner/pose_se2.hpp>
#include <teb_planner/obstacles.hpp>
#include <teb_planner/fake_ros_msgs.hpp>

namespace rip
{
    namespace navigation
    {
        namespace tebplanner
        {

            /**
             * @class BaseRobotFootprintModel
             * @brief Abstract class that defines the interface for robot footprint/contour models
             *
             * The robot model class is currently used in optimization only, since
             * taking the navigation stack footprint into account might be
             * inefficient. The footprint is only used for checking feasibility.
             */
            class BaseRobotFootprintModel
            {
            public:

                /**
                  * @brief Default constructor of the abstract obstacle class
                  */
                BaseRobotFootprintModel()
                {
                }

                /**
                 * @brief Virtual destructor.
                 */
                virtual ~BaseRobotFootprintModel()
                {
                }


                /**
                  * @brief Calculate the distance between the robot and an obstacle
                  * @param current_pose Current robot pose
                  * @param obstacle Pointer to the obstacle
                  * @return Euclidean distance to the robot
                  */
                virtual double calculateDistance(const PoseSE2& current_pose, std::shared_ptr<Obstacle> obstacle) const = 0;

                /**
                  * @brief Estimate the distance between the robot and the predicted location of an obstacle at time t
                  * @param current_pose robot pose, from which the distance to the obstacle is estimated
                  * @param obstacle Pointer to the dynamic obstacle (constant velocity model is assumed)
                  * @param t time, for which the predicted distance to the obstacle is calculated
                  * @return Euclidean distance to the robot
                  */
                virtual double estimateSpatioTemporalDistance(const PoseSE2& current_pose, std::shared_ptr<Obstacle> obstacle, double t) const = 0;

                /**
                 * @brief Compute the inscribed radius of the footprint model
                 * @return inscribed radius
                 */
                virtual double getInscribedRadius() = 0;

            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            };


            //! Abbrev. for shared obstacle pointers
            typedef std::shared_ptr<BaseRobotFootprintModel> RobotFootprintModelPtr;



            /**
             * @class PointRobotShape
             * @brief Class that defines a point-robot
             *
             * Instead of using a CircularRobotFootprint this class might
             * be utitilzed and the robot radius can be added to the mininum distance
             * parameter. This avoids a subtraction of zero each time a distance is calculated.
             */
            class PointRobotFootprint : public BaseRobotFootprintModel
            {
            public:

                /**
                  * @brief Default constructor of the abstract obstacle class
                  */
                PointRobotFootprint() {}

                /**
                 * @brief Virtual destructor.
                 */
                virtual ~PointRobotFootprint() {}

                /**
                  * @brief Calculate the distance between the robot and an obstacle
                  * @param current_pose Current robot pose
                  * @param obstacle Pointer to the obstacle
                  * @return Euclidean distance to the robot
                  */
                virtual double calculateDistance(const PoseSE2& current_pose, std::shared_ptr<Obstacle> obstacle) const
                {
                    return obstacle->getMinimumDistance(current_pose.position());
                }

                /**
                  * @brief Estimate the distance between the robot and the predicted location of an obstacle at time t
                  * @param current_pose robot pose, from which the distance to the obstacle is estimated
                  * @param obstacle Pointer to the dynamic obstacle (constant velocity model is assumed)
                  * @param t time, for which the predicted distance to the obstacle is calculated
                  * @return Euclidean distance to the robot
                  */
                virtual double estimateSpatioTemporalDistance(const PoseSE2& current_pose, std::shared_ptr<Obstacle> obstacle, double t) const
                {
                    return obstacle->getMinimumSpatioTemporalDistance(current_pose.position(), t);
                }

                /**
                 * @brief Compute the inscribed radius of the footprint model
                 * @return inscribed radius
                 */
                virtual double getInscribedRadius() {return 0.0;}

            };


            /**
             * @class CircularRobotFootprint
             * @brief Class that defines the a robot of circular shape
             */
            class CircularRobotFootprint : public BaseRobotFootprintModel
            {
            public:

                /**
                  * @brief Default constructor of the abstract obstacle class
                  * @param radius radius of the robot
                  */
                CircularRobotFootprint(double radius)
                    : m_radius(radius)
                { }

                /**
                 * @brief Virtual destructor.
                 */
                virtual ~CircularRobotFootprint()
                { }

                /**
                  * @brief Set radius of the circular robot
                  * @param radius radius of the robot
                  */
                void setRadius(double radius)
                {
                    m_radius = radius;
                }

                /**
                  * @brief Calculate the distance between the robot and an obstacle
                  * @param current_pose Current robot pose
                  * @param obstacle Pointer to the obstacle
                  * @return Euclidean distance to the robot
                  */
                virtual double calculateDistance(const PoseSE2& current_pose, std::shared_ptr<Obstacle> obstacle) const
                {
                    return obstacle->getMinimumDistance(current_pose.position()) - m_radius;
                }

                /**
                  * @brief Estimate the distance between the robot and the predicted location of an obstacle at time t
                  * @param current_pose robot pose, from which the distance to the obstacle is estimated
                  * @param obstacle Pointer to the dynamic obstacle (constant velocity model is assumed)
                  * @param t time, for which the predicted distance to the obstacle is calculated
                  * @return Euclidean distance to the robot
                  */
                virtual double estimateSpatioTemporalDistance(const PoseSE2& current_pose, std::shared_ptr<Obstacle> obstacle, double t) const
                {
                    return obstacle->getMinimumSpatioTemporalDistance(current_pose.position(), t) - m_radius;
                }

                /**
                 * @brief Compute the inscribed radius of the footprint model
                 * @return inscribed radius
                 */
                virtual double getInscribedRadius()
                {
                    return m_radius;
                }

            private:

                double m_radius;
            };


            /**
             * @class TwoCirclesRobotFootprint
             * @brief Class that approximates the robot with two shifted circles
             */
            class TwoCirclesRobotFootprint : public BaseRobotFootprintModel
            {
            public:

                /**
                  * @brief Default constructor of the abstract obstacle class
                  * @param front_offset shift the center of the front circle along the robot orientation starting from the center at the rear axis (in meters)
                  * @param front_radius radius of the front circle
                  * @param rear_offset shift the center of the rear circle along the opposite robot orientation starting from the center at the rear axis (in meters)
                  * @param rear_radius radius of the front circle
                  */
                TwoCirclesRobotFootprint(double front_offset, double front_radius, double rear_offset, double rear_radius)
                    : front_offset_(front_offset)
                    , front_radius_(front_radius)
                    , rear_offset_(rear_offset)
                    , rear_radius_(rear_radius)
                { }

                /**
                 * @brief Virtual destructor.
                 */
                virtual ~TwoCirclesRobotFootprint()
                { }

                /**
                 * @brief Set parameters of the contour/footprint
                 * @param front_offset shift the center of the front circle along the robot orientation starting from the center at the rear axis (in meters)
                 * @param front_radius radius of the front circle
                 * @param rear_offset shift the center of the rear circle along the opposite robot orientation starting from the center at the rear axis (in meters)
                 * @param rear_radius radius of the front circle
                 */
                void setParameters(double front_offset, double front_radius, double rear_offset, double rear_radius)
                {
                    front_offset_ = front_offset;
                    front_radius_ = front_radius;
                    rear_offset_ = rear_offset;
                    rear_radius_ = rear_radius;
                }

                /**
                  * @brief Calculate the distance between the robot and an obstacle
                  * @param current_pose Current robot pose
                  * @param obstacle Pointer to the obstacle
                  * @return Euclidean distance to the robot
                  */
                virtual double calculateDistance(const PoseSE2& current_pose, std::shared_ptr<Obstacle> obstacle) const
                {
                    Eigen::Vector2d dir = current_pose.orientationUnitVec();
                    double dist_front = obstacle->getMinimumDistance(current_pose.position() + front_offset_ * dir) - front_radius_;
                    double dist_rear = obstacle->getMinimumDistance(current_pose.position() - rear_offset_ * dir) - rear_radius_;
                    return std::min(dist_front, dist_rear);
                }

                /**
                  * @brief Estimate the distance between the robot and the predicted location of an obstacle at time t
                  * @param current_pose robot pose, from which the distance to the obstacle is estimated
                  * @param obstacle Pointer to the dynamic obstacle (constant velocity model is assumed)
                  * @param t time, for which the predicted distance to the obstacle is calculated
                  * @return Euclidean distance to the robot
                  */
                virtual double estimateSpatioTemporalDistance(const PoseSE2& current_pose, std::shared_ptr<Obstacle> obstacle, double t) const
                {
                    Eigen::Vector2d dir = current_pose.orientationUnitVec();
                    double dist_front = obstacle->getMinimumSpatioTemporalDistance(current_pose.position() + front_offset_ * dir, t) - front_radius_;
                    double dist_rear = obstacle->getMinimumSpatioTemporalDistance(current_pose.position() - rear_offset_ * dir, t) - rear_radius_;
                    return std::min(dist_front, dist_rear);
                }

                /**
                 * @brief Compute the inscribed radius of the footprint model
                 * @return inscribed radius
                 */
                virtual double getInscribedRadius()
                {
                    double min_longitudinal = std::min(rear_offset_ + rear_radius_, front_offset_ + front_radius_);
                    double min_lateral = std::min(rear_radius_, front_radius_);
                    return std::min(min_longitudinal, min_lateral);
                }

            private:

                double front_offset_;
                double front_radius_;
                double rear_offset_;
                double rear_radius_;

            };



            /**
             * @class LineRobotFootprint
             * @brief Class that approximates the robot with line segment (zero-width)
             */
            class LineRobotFootprint : public BaseRobotFootprintModel
            {
            public:

                /**
                  * @brief Default constructor of the abstract obstacle class
                  * @param line_start start coordinates (only x and y) of the line (w.r.t. robot center at (0,0))
                  * @param line_end end coordinates (only x and y) of the line (w.r.t. robot center at (0,0))
                  */
                LineRobotFootprint(const fakeros::Point& line_start, const fakeros::Point& line_end)
                {
                    setLine(line_start, line_end);
                }

                /**
                * @brief Default constructor of the abstract obstacle class (Eigen Version)
                * @param line_start start coordinates (only x and y) of the line (w.r.t. robot center at (0,0))
                * @param line_end end coordinates (only x and y) of the line (w.r.t. robot center at (0,0))
                */
                LineRobotFootprint(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end)
                {
                    setLine(line_start, line_end);
                }

                /**
                 * @brief Virtual destructor.
                 */
                virtual ~LineRobotFootprint() { }

                /**
                 * @brief Set vertices of the contour/footprint
                 * @param vertices footprint vertices (only x and y) around the robot center (0,0) (do not repeat the first and last vertex at the end)
                 */
                void setLine(const fakeros::Point& line_start, const fakeros::Point& line_end)
                {
                    m_start.x() = line_start.x;
                    m_start.y() = line_start.y;
                    m_end.x() = line_end.x;
                    m_end.y() = line_end.y;
                }

                /**
                 * @brief Set vertices of the contour/footprint (Eigen version)
                 * @param vertices footprint vertices (only x and y) around the robot center (0,0) (do not repeat the first and last vertex at the end)
                 */
                void setLine(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end)
                {
                    m_start = line_start;
                    m_end = line_end;
                }

                /**
                  * @brief Calculate the distance between the robot and an obstacle
                  * @param current_pose Current robot pose
                  * @param obstacle Pointer to the obstacle
                  * @return Euclidean distance to the robot
                  */
                virtual double calculateDistance(const PoseSE2& current_pose, std::shared_ptr<Obstacle> obstacle) const
                {
                    Eigen::Vector2d m_startworld;
                    Eigen::Vector2d m_endworld;
                    transformToWorld(current_pose, m_startworld, m_endworld);
                    return obstacle->getMinimumDistance(m_startworld, m_endworld);
                }

                /**
                  * @brief Estimate the distance between the robot and the predicted location of an obstacle at time t
                  * @param current_pose robot pose, from which the distance to the obstacle is estimated
                  * @param obstacle Pointer to the dynamic obstacle (constant velocity model is assumed)
                  * @param t time, for which the predicted distance to the obstacle is calculated
                  * @return Euclidean distance to the robot
                  */
                virtual double estimateSpatioTemporalDistance(const PoseSE2& current_pose, std::shared_ptr<Obstacle> obstacle, double t) const
                {
                    Eigen::Vector2d m_startworld;
                    Eigen::Vector2d m_endworld;
                    transformToWorld(current_pose, m_startworld, m_endworld);
                    return obstacle->getMinimumSpatioTemporalDistance(m_startworld, m_endworld, t);
                }

                /**
                 * @brief Compute the inscribed radius of the footprint model
                 * @return inscribed radius
                 */
                virtual double getInscribedRadius()
                {
                    return 0.0; // lateral distance = 0.0
                }

            private:

                /**
                  * @brief Transforms a line to the world frame manually
                  * @param current_pose Current robot pose
                  * @param[out] line_start m_start in the world frame
                  * @param[out] line_end m_end in the world frame
                  */
                void transformToWorld(const PoseSE2& current_pose, Eigen::Vector2d& m_startworld, Eigen::Vector2d& m_endworld) const
                {
                    double cos_th = std::cos(current_pose.theta());
                    double sin_th = std::sin(current_pose.theta());
                    m_startworld.x() = current_pose.x() + cos_th * m_start.x() - sin_th * m_start.y();
                    m_startworld.y() = current_pose.y() + sin_th * m_start.x() + cos_th * m_start.y();
                    m_endworld.x() = current_pose.x() + cos_th * m_end.x() - sin_th * m_end.y();
                    m_endworld.y() = current_pose.y() + sin_th * m_end.x() + cos_th * m_end.y();
                }

                Eigen::Vector2d m_start;
                Eigen::Vector2d m_end;

            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            };



            /**
             * @class PolygonRobotFootprint
             * @brief Class that approximates the robot with a closed polygon
             */
            class PolygonRobotFootprint : public BaseRobotFootprintModel
            {
            public:



                /**
                  * @brief Default constructor of the abstract obstacle class
                  * @param vertices footprint vertices (only x and y) around the robot center (0,0) (do not repeat the first and last vertex at the end)
                  */
                PolygonRobotFootprint(const Point2dContainer& vertices)
                    : m_vertices(vertices)
                { }

                /**
                 * @brief Virtual destructor.
                 */
                virtual ~PolygonRobotFootprint()
                { }

                /**
                 * @brief Set vertices of the contour/footprint
                 * @param vertices footprint vertices (only x and y) around the robot center (0,0) (do not repeat the first and last vertex at the end)
                 */
                void setVertices(const Point2dContainer& vertices)
                {
                    m_vertices = vertices;
                }

                /**
                  * @brief Calculate the distance between the robot and an obstacle
                  * @param current_pose Current robot pose
                  * @param obstacle Pointer to the obstacle
                  * @return Euclidean distance to the robot
                  */
                virtual double calculateDistance(const PoseSE2& current_pose, std::shared_ptr<Obstacle> obstacle) const
                {
                    Point2dContainer polygon_world(m_vertices.size());
                    transformToWorld(current_pose, polygon_world);
                    return obstacle->getMinimumDistance(polygon_world);
                }

                /**
                  * @brief Estimate the distance between the robot and the predicted location of an obstacle at time t
                  * @param current_pose robot pose, from which the distance to the obstacle is estimated
                  * @param obstacle Pointer to the dynamic obstacle (constant velocity model is assumed)
                  * @param t time, for which the predicted distance to the obstacle is calculated
                  * @return Euclidean distance to the robot
                  */
                virtual double estimateSpatioTemporalDistance(const PoseSE2& current_pose, std::shared_ptr<Obstacle> obstacle, double t) const
                {
                    Point2dContainer polygon_world(m_vertices.size());
                    transformToWorld(current_pose, polygon_world);
                    return obstacle->getMinimumSpatioTemporalDistance(polygon_world, t);
                }

                /**
                 * @brief Compute the inscribed radius of the footprint model
                 * @return inscribed radius
                 */
                virtual double getInscribedRadius()
                {
                    double min_dist = std::numeric_limits<double>::max();
                    Eigen::Vector2d center(0.0, 0.0);

                    if (m_vertices.size() <= 2)
                    {
                        return 0.0;
                    }

                    for (int i = 0; i < (int)m_vertices.size() - 1; ++i)
                    {
                        // compute distance from the robot center point to the first vertex
                        double vertex_dist = m_vertices[i].norm();
                        double edge_dist = distance_point_to_segment_2d(center, m_vertices[i], m_vertices[i + 1]);
                        min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
                    }

                    // we also need to check the last vertex and the first vertex
                    double vertex_dist = m_vertices.back().norm();
                    double edge_dist = distance_point_to_segment_2d(center, m_vertices.back(), m_vertices.front());
                    return std::min(min_dist, std::min(vertex_dist, edge_dist));
                }

            private:

                /**
                  * @brief Transforms a polygon to the world frame manually
                  * @param current_pose Current robot pose
                  * @param[out] polygon_world polygon in the world frame
                  */
                void transformToWorld(const PoseSE2& current_pose, Point2dContainer& polygon_world) const
                {
                    double cos_th = std::cos(current_pose.theta());
                    double sin_th = std::sin(current_pose.theta());
                    for (std::size_t i = 0; i < m_vertices.size(); ++i)
                    {
                        polygon_world[i].x() = current_pose.x() + cos_th * m_vertices[i].x() - sin_th * m_vertices[i].y();
                        polygon_world[i].y() = current_pose.y() + sin_th * m_vertices[i].x() + cos_th * m_vertices[i].y();
                    }
                }
                Point2dContainer m_vertices;
            };
        }
    }
}

#endif // ROBOT_FOOTPRINT_MODEL_HPP
