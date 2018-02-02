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

#ifndef POSE_SE2_HPP
#define POSE_SE2_HPP

#include <g2o/stuff/misc.h>

#include <eigen3/Eigen/Core>
#include <teb_planner/misc.hpp>
#include <teb_planner/fake_ros_msgs.hpp>

namespace rip
{
    namespace navigation
    {
        namespace tebplanner
        {

            /**
              * @class PoseSE2
              * @brief This class implements a pose in the domain SE2: \f$ \mathbb{R}^2 \times S^1 \f$
              * The pose consist of the position x and y and an orientation given as angle theta [-pi, pi].
              */
            class PoseSE2
            {
            public:

                /** @name Construct PoseSE2 instances */
                ///@{

                /**
                  * @brief Default constructor
                  */
                PoseSE2()
                {
                    setZero();
                }

                /**
                  * @brief Construct pose given a position vector and an angle theta
                  * @param position 2D position vector
                  * @param theta angle given in rad
                  */
                PoseSE2(const Eigen::Ref<const Eigen::Vector2d>& position, double theta)
                {
                    m_position = position;
                    m_theta = theta;
                }

                /**
                  * @brief Construct pose using single components x, y, and the yaw angle
                  * @param x x-coordinate
                  * @param y y-coordinate
                  * @param theta yaw angle in rad
                  */
                PoseSE2(double x, double y, double theta)
                {
                    m_position.coeffRef(0) = x;
                    m_position.coeffRef(1) = y;
                    m_theta = theta;
                }

                /**
                 * @brief Construct pose using a geometry_msgs::Pose
                 * @param pose geometry_msgs::Pose object
                 */
                PoseSE2(const fakeros::Pose& pose)
                {
                    m_position.coeffRef(0) = pose.position.x;
                    m_position.coeffRef(1) = pose.position.y;
                    m_theta = asin(2 * pose.orientation.x * pose.orientation.y + 2 * pose.orientation.z * pose.orientation.w);
                }

                /**
                  * @brief Copy constructor
                  * @param pose PoseSE2 instance
                  */
                PoseSE2(const PoseSE2& pose)
                {
                    m_position = pose.m_position;
                    m_theta = pose.m_theta;
                }

                ///@}


                /**
                  * @brief Destructs the PoseSE2
                  */
                ~PoseSE2()
                {}


                /** @name Access and modify values */
                ///@{

                /**
                  * @brief Access the 2D position part
                  * @see estimate
                  * @return reference to the 2D position part
                  */
                Eigen::Vector2d& position()
                {
                    return m_position;
                }

                /**
                  * @brief Access the 2D position part (read-only)
                  * @see estimate
                  * @return const reference to the 2D position part
                  */
                const Eigen::Vector2d& position() const
                {
                    return m_position;
                }

                /**
                  * @brief Access the x-coordinate the pose
                  * @return reference to the x-coordinate
                  */
                double& x()
                {
                    return m_position.coeffRef(0);
                }

                /**
                  * @brief Access the x-coordinate the pose (read-only)
                  * @return const reference to the x-coordinate
                  */
                const double& x() const
                {
                    return m_position.coeffRef(0);
                }

                /**
                  * @brief Access the y-coordinate the pose
                  * @return reference to the y-coordinate
                  */
                double& y()
                {
                    return m_position.coeffRef(1);
                }

                /**
                  * @brief Access the y-coordinate the pose (read-only)
                  * @return const reference to the y-coordinate
                  */
                const double& y() const
                {
                    return m_position.coeffRef(1);
                }

                /**
                  * @brief Access the orientation part (yaw angle) of the pose
                  * @return reference to the yaw angle
                  */
                double& theta()
                {
                    return m_theta;
                }

                /**
                  * @brief Access the orientation part (yaw angle) of the pose (read-only)
                  * @return const reference to the yaw angle
                  */
                const double& theta() const
                {
                    return m_theta;
                }

                /**
                  * @brief Set pose to [0,0,0]
                  */
                void setZero()
                {
                    m_position.setZero();
                    m_theta = 0;
                }

                /**
                   * @brief Convert PoseSE2 to a geometry_msgs::Pose
                   * @param[out] pose Pose message
                   */
                void toPoseMsg(fakeros::Pose& pose) const
                {
                    pose.position.x = m_position.x();
                    pose.position.y = m_position.y();
                    pose.position.z = 0;

                    pose.orientation.x = cos(m_theta / 2.0);
                    pose.orientation.y = 0;
                    pose.orientation.z = 0;
                    pose.orientation.w = sin(m_theta / 2.0);
                }

                /**
                 * @brief Return the unit vector of the current orientation
                 * @returns [cos(theta), sin(theta))]^T
                 */
                Eigen::Vector2d orientationUnitVec() const
                {
                    return Eigen::Vector2d(std::cos(m_theta), std::sin(m_theta));
                }

                ///@}


                /** @name Arithmetic operations for which operators are not always reasonable */
                ///@{

                /**
                  * @brief Scale all SE2 components (x,y,theta) and normalize theta afterwards to [-pi, pi]
                  * @param factor scale factor
                  */
                void scale(double factor)
                {
                    m_position *= factor;
                    m_theta = g2o::normalize_theta( m_theta * factor );
                }

                /**
                  * @brief Increment the pose by adding a double[3] array
                  * The angle is normalized afterwards
                  * @param pose_as_array 3D double array [x, y, theta]
                  */
                void plus(const double* pose_as_array)
                {
                    m_position.coeffRef(0) += pose_as_array[0];
                    m_position.coeffRef(1) += pose_as_array[1];
                    m_theta = g2o::normalize_theta( m_theta + pose_as_array[2] );
                }

                /**
                  * @brief Get the mean / average of two poses and store it in the caller class
                  * For the position part: 0.5*(x1+x2)
                  * For the angle: take the angle of the mean direction vector
                  * @param pose1 first pose to consider
                  * @param pose2 second pose to consider
                  */
                void averageInPlace(const PoseSE2& pose1, const PoseSE2& pose2)
                {
                    m_position = (pose1.m_position + pose2.m_position) / 2;
                    m_theta = g2o::average_angle(pose1.m_theta, pose2.m_theta);
                }

                /**
                  * @brief Get the mean / average of two poses and return the result (static)
                  * For the position part: 0.5*(x1+x2)
                  * For the angle: take the angle of the mean direction vector
                  * @param pose1 first pose to consider
                  * @param pose2 second pose to consider
                  * @return mean / average of \c pose1 and \c pose2
                  */
                static PoseSE2 average(const PoseSE2& pose1, const PoseSE2& pose2)
                {
                    return PoseSE2( (pose1.m_position + pose2.m_position) / 2 , g2o::average_angle(pose1.m_theta, pose2.m_theta) );
                }

                /**
                  * @brief Rotate pose globally
                  *
                  * Compute [pose_x, pose_y] = Rot(\c angle) * [pose_x, pose_y].
                  * if \c adjust_theta, pose_theta is also rotated by \c angle
                  * @param angle the angle defining the 2d rotation
                  * @param adjust_theta if \c true, the orientation theta is also rotated
                  */
                void rotateGlobal(double angle, bool adjust_theta = true)
                {
                    double new_x = std::cos(angle) * m_position.x() - std::sin(angle) * m_position.y();
                    double new_y = std::sin(angle) * m_position.x() + std::cos(angle) * m_position.y();
                    m_position.x() = new_x;
                    m_position.y() = new_y;
                    if (adjust_theta)
                    {
                        m_theta = g2o::normalize_theta(m_theta + angle);
                    }
                }

                ///@}


                /** @name Operator overloads / Allow some arithmetic operations */
                ///@{

                /**
                  * @brief Asignment operator
                  * @param rhs PoseSE2 instance
                  * @todo exception safe version of the assignment operator
                  */
                PoseSE2& operator=( const PoseSE2& rhs )
                {
                    if (&rhs != this)
                    {
                        m_position = rhs.m_position;
                        m_theta = rhs.m_theta;
                    }
                    return *this;
                }

                /**
                  * @brief Compound assignment operator (addition)
                  * @param rhs addend
                  */
                PoseSE2& operator+=(const PoseSE2& rhs)
                {
                    m_position += rhs.m_position;
                    m_theta = g2o::normalize_theta(m_theta + rhs.m_theta);
                    return *this;
                }

                /**
                * @brief Arithmetic operator overload for additions
                * @param lhs First addend
                * @param rhs Second addend
                */
                friend PoseSE2 operator+(PoseSE2 lhs, const PoseSE2& rhs)
                {
                    return lhs += rhs;
                }

                /**
                  * @brief Compound assignment operator (subtraction)
                  * @param rhs value to subtract
                  */
                PoseSE2& operator-=(const PoseSE2& rhs)
                {
                    m_position -= rhs.m_position;
                    m_theta = g2o::normalize_theta(m_theta - rhs.m_theta);
                    return *this;
                }

                /**
                * @brief Arithmetic operator overload for subtractions
                * @param lhs First term
                * @param rhs Second term
                */
                friend PoseSE2 operator-(PoseSE2 lhs, const PoseSE2& rhs)
                {
                    return lhs -= rhs;
                }

                /**
                  * @brief Multiply pose with scalar and return copy without normalizing theta
                  * This operator is useful for calculating velocities ...
                  * @param pose pose to scale
                  * @param scalar factor to multiply with
                  * @warning theta is not normalized after multiplying
                  */
                friend PoseSE2 operator*(PoseSE2 pose, double scalar)
                {
                    pose.m_position *= scalar;
                    pose.m_theta *= scalar;
                    return pose;
                }

                /**
                  * @brief Multiply pose with scalar and return copy without normalizing theta
                  * This operator is useful for calculating velocities ...
                  * @param scalar factor to multiply with
                  * @param pose pose to scale
                  * @warning theta is not normalized after multiplying
                  */
                friend PoseSE2 operator*(double scalar, PoseSE2 pose)
                {
                    pose.m_position *= scalar;
                    pose.m_theta *= scalar;
                    return pose;
                }

                /**
                 * @brief Output stream operator
                 * @param stream output stream
                 * @param pose to be used
                 */
                friend std::ostream& operator<< (std::ostream& stream, const PoseSE2& pose)
                {
                    stream << "x: " << pose.m_position[0] << " y: " << pose.m_position[1] << " theta: " << pose.m_theta;
                    return stream;
                }

                ///@}
            private:
                Eigen::Vector2d m_position;
                double m_theta;

            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            };


        }
    }
}

#endif // POSE_SE2_H_
