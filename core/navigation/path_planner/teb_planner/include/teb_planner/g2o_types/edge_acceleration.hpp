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
 * Notes:
 * The following class is derived from a class defined by the
 * g2o-framework. g2o is licensed under the terms of the BSD License.
 * Refer to the base class source for detailed licensing information.
 *
 * Author: Christoph Rösmann
 *
 * Modified by: Andrew Messing
 * - Removed all aspects that required ROS or boost and added in RIP
 *   elements
 *********************************************************************/



#ifndef EDGE_ACCELERATION_HPP
#define EDGE_ACCELERATION_HPP

#include <misc/logger.hpp>

#include <teb_planner/g2o_types/vertex_pose.hpp>
#include <teb_planner/g2o_types/vertex_time_diff.hpp>
#include <teb_planner/g2o_types/penalties.hpp>
#include <teb_planner/teb_config.hpp>
#include <teb_planner/g2o_types/base_teb_edges.hpp>
#include <teb_planner/fake_ros_msgs.hpp>

namespace rip
{
    namespace navigation
    {
        namespace tebplanner
        {

            /**
             * @class EdgeAcceleration
             * @brief Edge defining the cost function for limiting the translational and rotational acceleration.
             *
             * The edge depends on five vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \mathbf{s}_{ip2}, \Delta T_i, \Delta T_{ip1} \f$ and minimizes:
             * \f$ \min \textrm{penaltyInterval}( [a, omegadot } ]^T ) \cdot weight \f$. \n
             * \e a is calculated using the difference quotient (twice) and the position parts of all three poses \n
             * \e omegadot is calculated using the difference quotient of the yaw angles followed by a normalization to [-pi, pi]. \n
             * \e weight can be set using setInformation() \n
             * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval() \n
             * The dimension of the error / cost vector is 2: the first component represents the translational acceleration and
             * the second one the rotational acceleration.
             * @see TebOptimalPlanner::AddEdgesAcceleration
             * @see EdgeAccelerationStart
             * @see EdgeAccelerationGoal
             * @remarks Do not forget to call setTebConfig()
             * @remarks Refer to EdgeAccelerationStart() and EdgeAccelerationGoal() for defining boundary values!
             */
            class EdgeAcceleration : public BaseTebMultiEdge<2, double>
            {
            public:

                /**
                 * @brief Construct edge.
                 */
                EdgeAcceleration()
                {
                    this->resize(5);
                }

                /**
                 * @brief Actual cost function
                 */
                void computeError()
                {
                    if (!m_config)
                    {
                        misc::Logger::getInstance()->error("You must call setTebConfig() on EdgeAcceleration()");
                        assert(false);
                    }

                    const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
                    const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
                    const VertexPose* pose3 = static_cast<const VertexPose*>(_vertices[2]);
                    const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[3]);
                    const VertexTimeDiff* dt2 = static_cast<const VertexTimeDiff*>(_vertices[4]);

                    // VELOCITY & ACCELERATION
                    const Eigen::Vector2d diff1 = pose2->position() - pose1->position();
                    const Eigen::Vector2d diff2 = pose3->position() - pose2->position();

                    double dist1 = diff1.norm();
                    double dist2 = diff2.norm();
                    const double angle_diff1 = g2o::normalize_theta(pose2->theta() - pose1->theta());
                    const double angle_diff2 = g2o::normalize_theta(pose3->theta() - pose2->theta());

                    if (m_config->trajectory.exact_arc_length) // use exact arc length instead of Euclidean approximation
                    {
                        if (angle_diff1 != 0)
                        {
                            const double radius =  dist1 / (2 * sin(angle_diff1 / 2));
                            dist1 = fabs( angle_diff1 * radius ); // actual arc length!
                        }
                        if (angle_diff2 != 0)
                        {
                            const double radius =  dist2 / (2 * sin(angle_diff2 / 2));
                            dist2 = fabs( angle_diff2 * radius ); // actual arc length!
                        }
                    }

                    double vel1 = dist1 / dt1->dt();
                    double vel2 = dist2 / dt2->dt();


                    // consider directions
                    vel1 *= fast_sigmoid( 100 * (diff1.x() * cos(pose1->theta()) + diff1.y() * sin(pose1->theta())) );
                    vel2 *= fast_sigmoid( 100 * (diff2.x() * cos(pose2->theta()) + diff2.y() * sin(pose2->theta())) );

                    const double acc_lin  = (vel2 - vel1) * 2 / ( dt1->dt() + dt2->dt() );


                    _error[0] = penaltyBoundToInterval(acc_lin, m_config->robot.acc_lim_x, m_config->optim.penalty_epsilon);

                    // ANGULAR ACCELERATION
                    const double omega1 = angle_diff1 / dt1->dt();
                    const double omega2 = angle_diff2 / dt2->dt();
                    const double acc_rot  = (omega2 - omega1) * 2 / ( dt1->dt() + dt2->dt() );

                    _error[1] = penaltyBoundToInterval(acc_rot, m_config->robot.acc_lim_theta, m_config->optim.penalty_epsilon);


                    if (!std::isfinite(_error[0]))
                    {
                        misc::Logger::getInstance()->error(fmt::format("EdgeAccelerationStart::computeError() translational: _error[0]={}\n", _error[0]));
                        assert(false);
                    }
                    if (!std::isfinite(_error[1]))
                    {
                        misc::Logger::getInstance()->error(fmt::format("EdgeAccelerationStart::computeError() rotational: _error[1]={}\n", _error[1]));
                        assert(false);
                    }
                }


                EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            };


            /**
             * @class EdgeAccelerationStart
             * @brief Edge defining the cost function for limiting the translational and rotational acceleration at the beginning of the trajectory.
             *
             * The edge depends on three vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \Delta T_i \f$, an initial velocity defined by setInitialVelocity()
             * and minimizes: \n
             * \f$ \min \textrm{penaltyInterval}( [a, omegadot ]^T ) \cdot weight \f$. \n
             * \e a is calculated using the difference quotient (twice) and the position parts of the poses. \n
             * \e omegadot is calculated using the difference quotient of the yaw angles followed by a normalization to [-pi, pi].  \n
             * \e weight can be set using setInformation(). \n
             * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval(). \n
             * The dimension of the error / cost vector is 2: the first component represents the translational acceleration and
             * the second one the rotational acceleration.
             * @see TebOptimalPlanner::AddEdgesAcceleration
             * @see EdgeAcceleration
             * @see EdgeAccelerationGoal
             * @remarks Do not forget to call setTebConfig()
             * @remarks Refer to EdgeAccelerationGoal() for defining boundary values at the end of the trajectory!
             */
            class EdgeAccelerationStart : public BaseTebMultiEdge<2, const fakeros::Twist*>
            {
            public:

                /**
                 * @brief Construct edge.
                 */
                EdgeAccelerationStart()
                {
                    _measurement = nullptr;
                    this->resize(3);
                }


                /**
                 * @brief Actual cost function
                 */
                void computeError()
                {
                    if (!m_config || !_measurement)
                    {
                        misc::Logger::getInstance()->error("You must call setTebConfig() and setStartVelocity() on EdgeAccelerationStart()");
                        assert(false);
                    }

                    const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
                    const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
                    const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);

                    // VELOCITY & ACCELERATION
                    const Eigen::Vector2d diff = pose2->position() - pose1->position();
                    double dist = diff.norm();
                    const double angle_diff = g2o::normalize_theta(pose2->theta() - pose1->theta());
                    if (m_config->trajectory.exact_arc_length && angle_diff != 0)
                    {
                        const double radius =  dist / (2 * sin(angle_diff / 2));
                        dist = fabs( angle_diff * radius ); // actual arg length!
                    }

                    const double vel1 = _measurement->linear.x;
                    double vel2 = dist / dt->dt();

                    // consider directions
                    vel2 *= fast_sigmoid( 100 * (diff.x() * cos(pose1->theta()) + diff.y() * sin(pose1->theta())) );

                    const double acc_lin  = (vel2 - vel1) / dt->dt();

                    _error[0] = penaltyBoundToInterval(acc_lin, m_config->robot.acc_lim_x, m_config->optim.penalty_epsilon);

                    // ANGULAR ACCELERATION
                    const double omega1 = _measurement->angular.z;
                    const double omega2 = angle_diff / dt->dt();
                    const double acc_rot  = (omega2 - omega1) / dt->dt();

                    _error[1] = penaltyBoundToInterval(acc_rot, m_config->robot.acc_lim_theta, m_config->optim.penalty_epsilon);

                    if (!std::isfinite(_error[0]))
                    {
                        misc::Logger::getInstance()->error(fmt::format("EdgeAccelerationStart::computeError() translational: _error[0]={}\n", _error[0]));
                        assert(false);
                    }
                    if (!std::isfinite(_error[1]))
                    {
                        misc::Logger::getInstance()->error(fmt::format("EdgeAccelerationStart::computeError() rotational: _error[1]={}\n", _error[1]));
                        assert(false);
                    }
                }

                /**
                 * @brief Set the initial velocity that is taken into account for calculating the acceleration
                 * @param vel_start twist message containing the translational and rotational velocity
                 */
                void setInitialVelocity(const fakeros::Twist& vel_start)
                {
                    _measurement = &vel_start;
                }

                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            };




            /**
             * @class EdgeAccelerationGoal
             * @brief Edge defining the cost function for limiting the translational and rotational acceleration at the end of the trajectory.
             *
             * The edge depends on three vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \Delta T_i \f$, an initial velocity defined by setGoalVelocity()
             * and minimizes: \n
             * \f$ \min \textrm{penaltyInterval}( [a, omegadot ]^T ) \cdot weight \f$. \n
             * \e a is calculated using the difference quotient (twice) and the position parts of the poses \n
             * \e omegadot is calculated using the difference quotient of the yaw angles followed by a normalization to [-pi, pi].  \n
             * \e weight can be set using setInformation() \n
             * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval() \n
             * The dimension of the error / cost vector is 2: the first component represents the translational acceleration and
             * the second one the rotational acceleration.
             * @see TebOptimalPlanner::AddEdgesAcceleration
             * @see EdgeAcceleration
             * @see EdgeAccelerationStart
             * @remarks Do not forget to call setTebConfig()
             * @remarks Refer to EdgeAccelerationStart() for defining boundary (initial) values at the end of the trajectory
             */
            class EdgeAccelerationGoal : public BaseTebMultiEdge<2, const fakeros::Twist*>
            {
            public:

                /**
                 * @brief Construct edge.
                 */
                EdgeAccelerationGoal()
                {
                    _measurement = nullptr;
                    this->resize(3);
                }


                /**
                 * @brief Actual cost function
                 */
                void computeError()
                {
                    if (!m_config || !_measurement)
                    {
                        misc::Logger::getInstance()->error("You must call setTebConfig() and setGoalVelocity() on EdgeAccelerationGoal()");
                        assert(false);
                    }

                    const VertexPose* pose_pre_goal = static_cast<const VertexPose*>(_vertices[0]);
                    const VertexPose* pose_goal = static_cast<const VertexPose*>(_vertices[1]);
                    const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);

                    // VELOCITY & ACCELERATION

                    const Eigen::Vector2d diff = pose_goal->position() - pose_pre_goal->position();
                    double dist = diff.norm();
                    const double angle_diff = g2o::normalize_theta(pose_goal->theta() - pose_pre_goal->theta());
                    if (m_config->trajectory.exact_arc_length  && angle_diff != 0)
                    {
                        double radius =  dist / (2 * sin(angle_diff / 2));
                        dist = fabs( angle_diff * radius ); // actual arg length!
                    }

                    double vel1 = dist / dt->dt();
                    const double vel2 = _measurement->linear.x;

                    // consider directions
                    vel1 *= fast_sigmoid( 100 * (diff.x() * cos(pose_pre_goal->theta()) + diff.y() * sin(pose_pre_goal->theta())) );

                    const double acc_lin  = (vel2 - vel1) / dt->dt();

                    _error[0] = penaltyBoundToInterval(acc_lin, m_config->robot.acc_lim_x, m_config->optim.penalty_epsilon);

                    // ANGULAR ACCELERATION
                    const double omega1 = angle_diff / dt->dt();
                    const double omega2 = _measurement->angular.z;
                    const double acc_rot  = (omega2 - omega1) / dt->dt();

                    _error[1] = penaltyBoundToInterval(acc_rot, m_config->robot.acc_lim_theta, m_config->optim.penalty_epsilon);

                    if (!std::isfinite(_error[0]))
                    {
                        misc::Logger::getInstance()->error(fmt::format("EdgeAccelerationStart::computeError() translational: _error[0]={}\n", _error[0]));
                        assert(false);
                    }
                    if (!std::isfinite(_error[1]))
                    {
                        misc::Logger::getInstance()->error(fmt::format("EdgeAccelerationStart::computeError() rotational: _error[1]={}\n", _error[1]));
                        assert(false);
                    }
                }

                /**
                 * @brief Set the goal / final velocity that is taken into account for calculating the acceleration
                 * @param vel_goal twist message containing the translational and rotational velocity
                 */
                void setGoalVelocity(const fakeros::Twist& vel_goal)
                {
                    _measurement = &vel_goal;
                }

                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            };




            /**
             * @class EdgeAccelerationHolonomic
             * @brief Edge defining the cost function for limiting the translational and rotational acceleration.
             *
             * The edge depends on five vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \mathbf{s}_{ip2}, \Delta T_i, \Delta T_{ip1} \f$ and minimizes:
             * \f$ \min \textrm{penaltyInterval}( [ax, ay, omegadot } ]^T ) \cdot weight \f$. \n
             * \e ax is calculated using the difference quotient (twice) and the x position parts of all three poses \n
             * \e ay is calculated using the difference quotient (twice) and the y position parts of all three poses \n
             * \e omegadot is calculated using the difference quotient of the yaw angles followed by a normalization to [-pi, pi]. \n
             * \e weight can be set using setInformation() \n
             * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval() \n
             * The dimension of the error / cost vector is 3: the first component represents the translational acceleration (x-dir),
             * the second one the strafing acceleration and the third one the rotational acceleration.
             * @see TebOptimalPlanner::AddEdgesAcceleration
             * @see EdgeAccelerationHolonomicStart
             * @see EdgeAccelerationHolonomicGoal
             * @remarks Do not forget to call setTebConfig()
             * @remarks Refer to EdgeAccelerationHolonomicStart() and EdgeAccelerationHolonomicGoal() for defining boundary values!
             */
            class EdgeAccelerationHolonomic : public BaseTebMultiEdge<3, double>
            {
            public:

                /**
                 * @brief Construct edge.
                 */
                EdgeAccelerationHolonomic()
                {
                    this->resize(5);
                }

                /**
                 * @brief Actual cost function
                 */
                void computeError()
                {
                    if (!m_config)
                    {
                        misc::Logger::getInstance()->error("You must call setTebConfig() on EdgeAccelerationHolonomic()");
                        assert(false);
                    }

                    const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
                    const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
                    const VertexPose* pose3 = static_cast<const VertexPose*>(_vertices[2]);
                    const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[3]);
                    const VertexTimeDiff* dt2 = static_cast<const VertexTimeDiff*>(_vertices[4]);

                    // VELOCITY & ACCELERATION
                    Eigen::Vector2d diff1 = pose2->position() - pose1->position();
                    Eigen::Vector2d diff2 = pose3->position() - pose2->position();

                    double cos_theta1 = std::cos(pose1->theta());
                    double sin_theta1 = std::sin(pose1->theta());
                    double cos_theta2 = std::cos(pose2->theta());
                    double sin_theta2 = std::sin(pose2->theta());

                    // transform pose2 into robot frame pose1 (inverse 2d rotation matrix)
                    double p1_dx =  cos_theta1 * diff1.x() + sin_theta1 * diff1.y();
                    double p1_dy = -sin_theta1 * diff1.x() + cos_theta1 * diff1.y();
                    // transform pose3 into robot frame pose2 (inverse 2d rotation matrix)
                    double p2_dx =  cos_theta2 * diff2.x() + sin_theta2 * diff2.y();
                    double p2_dy = -sin_theta2 * diff2.x() + cos_theta2 * diff2.y();

                    double vel1_x = p1_dx / dt1->dt();
                    double vel1_y = p1_dy / dt1->dt();
                    double vel2_x = p2_dx / dt2->dt();
                    double vel2_y = p2_dy / dt2->dt();

                    double dt12 = dt1->dt() + dt2->dt();

                    double acc_x  = (vel2_x - vel1_x) * 2 / dt12;
                    double acc_y  = (vel2_y - vel1_y) * 2 / dt12;

                    _error[0] = penaltyBoundToInterval(acc_x, m_config->robot.acc_lim_x, m_config->optim.penalty_epsilon);
                    _error[1] = penaltyBoundToInterval(acc_y, m_config->robot.acc_lim_y, m_config->optim.penalty_epsilon);

                    // ANGULAR ACCELERATION
                    double omega1 = g2o::normalize_theta(pose2->theta() - pose1->theta()) / dt1->dt();
                    double omega2 = g2o::normalize_theta(pose3->theta() - pose2->theta()) / dt2->dt();
                    double acc_rot  = (omega2 - omega1) * 2 / dt12;

                    _error[2] = penaltyBoundToInterval(acc_rot, m_config->robot.acc_lim_theta, m_config->optim.penalty_epsilon);

                    if (!std::isfinite(_error[0]))
                    {
                        misc::Logger::getInstance()->error(fmt::format("EdgeAccelerationStart::computeError() translational: _error[0]={}\n", _error[0]));
                        assert(false);
                    }
                    if (!std::isfinite(_error[1]))
                    {
                        misc::Logger::getInstance()->error(fmt::format("EdgeAccelerationStart::computeError() strafing: _error[1]={}\n", _error[1]));
                        assert(false);
                    }
                    if (!std::isfinite(_error[2]))
                    {
                        misc::Logger::getInstance()->error(fmt::format("EdgeAccelerationStart::computeError() rotational: _error[2]={}\n", _error[2]));
                        assert(false);
                    }
                }

                EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            };


            /**
             * @class EdgeAccelerationHolonomicStart
             * @brief Edge defining the cost function for limiting the translational and rotational acceleration at the beginning of the trajectory.
             *
             * The edge depends on three vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \Delta T_i \f$, an initial velocity defined by setInitialVelocity()
             * and minimizes: \n
             * \f$ \min \textrm{penaltyInterval}( [ax, ay, omegadot ]^T ) \cdot weight \f$. \n
             * \e ax is calculated using the difference quotient (twice) and the x-position parts of the poses. \n
             * \e ay is calculated using the difference quotient (twice) and the y-position parts of the poses. \n
             * \e omegadot is calculated using the difference quotient of the yaw angles followed by a normalization to [-pi, pi].  \n
             * \e weight can be set using setInformation(). \n
             * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval(). \n
             * The dimension of the error / cost vector is 3: the first component represents the translational acceleration,
             * the second one the strafing acceleration and the third one the rotational acceleration.
             * @see TebOptimalPlanner::AddEdgesAcceleration
             * @see EdgeAccelerationHolonomic
             * @see EdgeAccelerationHolonomicGoal
             * @remarks Do not forget to call setTebConfig()
             * @remarks Refer to EdgeAccelerationHolonomicGoal() for defining boundary values at the end of the trajectory!
             */
            class EdgeAccelerationHolonomicStart : public BaseTebMultiEdge<3, const fakeros::Twist*>
            {
            public:

                /**
                 * @brief Construct edge.
                 */
                EdgeAccelerationHolonomicStart()
                {
                    this->resize(3);
                    _measurement = nullptr;
                }

                /**
                 * @brief Actual cost function
                 */
                void computeError()
                {

                    if (!m_config || !_measurement)
                    {
                        misc::Logger::getInstance()->error("You must call setTebConfig() and setStartVelocity() on EdgeAccelerationHolonomicStart()");
                        assert(false);
                    }

                    const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
                    const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
                    const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);

                    // VELOCITY & ACCELERATION
                    Eigen::Vector2d diff = pose2->position() - pose1->position();

                    double cos_theta1 = std::cos(pose1->theta());
                    double sin_theta1 = std::sin(pose1->theta());

                    // transform pose2 into robot frame pose1 (inverse 2d rotation matrix)
                    double p1_dx =  cos_theta1 * diff.x() + sin_theta1 * diff.y();
                    double p1_dy = -sin_theta1 * diff.x() + cos_theta1 * diff.y();

                    double vel1_x = _measurement->linear.x;
                    double vel1_y = _measurement->linear.y;
                    double vel2_x = p1_dx / dt->dt();
                    double vel2_y = p1_dy / dt->dt();

                    double acc_lin_x  = (vel2_x - vel1_x) / dt->dt();
                    double acc_lin_y  = (vel2_y - vel1_y) / dt->dt();

                    _error[0] = penaltyBoundToInterval(acc_lin_x, m_config->robot.acc_lim_x, m_config->optim.penalty_epsilon);
                    _error[1] = penaltyBoundToInterval(acc_lin_y, m_config->robot.acc_lim_y, m_config->optim.penalty_epsilon);

                    // ANGULAR ACCELERATION
                    double omega1 = _measurement->angular.z;
                    double omega2 = g2o::normalize_theta(pose2->theta() - pose1->theta()) / dt->dt();
                    double acc_rot  = (omega2 - omega1) / dt->dt();

                    _error[2] = penaltyBoundToInterval(acc_rot, m_config->robot.acc_lim_theta, m_config->optim.penalty_epsilon);

                    if (!std::isfinite(_error[0]))
                    {
                        misc::Logger::getInstance()->error(fmt::format("EdgeAccelerationStart::computeError() translational: _error[0]={}\n", _error[0]));
                        assert(false);
                    }
                    if (!std::isfinite(_error[1]))
                    {
                        misc::Logger::getInstance()->error(fmt::format("EdgeAccelerationStart::computeError() strafing: _error[1]={}\n", _error[1]));
                        assert(false);
                    }
                    if (!std::isfinite(_error[2]))
                    {
                        misc::Logger::getInstance()->error(fmt::format("EdgeAccelerationStart::computeError() rotational: _error[2]={}\n", _error[2]));
                        assert(false);
                    }
                }

                /**
                 * @brief Set the initial velocity that is taken into account for calculating the acceleration
                 * @param vel_start twist message containing the translational and rotational velocity
                 */
                void setInitialVelocity(const fakeros::Twist& vel_start)
                {
                    _measurement = &vel_start;
                }

                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            };



            /**
             * @class EdgeAccelerationHolonomicGoal
             * @brief Edge defining the cost function for limiting the translational and rotational acceleration at the end of the trajectory.
             *
             * The edge depends on three vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \Delta T_i \f$, an initial velocity defined by setGoalVelocity()
             * and minimizes: \n
             * \f$ \min \textrm{penaltyInterval}( [ax, ay, omegadot ]^T ) \cdot weight \f$. \n
             * \e ax is calculated using the difference quotient (twice) and the x-position parts of the poses \n
             * \e ay is calculated using the difference quotient (twice) and the y-position parts of the poses \n
             * \e omegadot is calculated using the difference quotient of the yaw angles followed by a normalization to [-pi, pi].  \n
             * \e weight can be set using setInformation() \n
             * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval() \n
             * The dimension of the error / cost vector is 3: the first component represents the translational acceleration,
             * the second one is the strafing velocity and the third one the rotational acceleration.
             * @see TebOptimalPlanner::AddEdgesAcceleration
             * @see EdgeAccelerationHolonomic
             * @see EdgeAccelerationHolonomicStart
             * @remarks Do not forget to call setTebConfig()
             * @remarks Refer to EdgeAccelerationHolonomicStart() for defining boundary (initial) values at the end of the trajectory
             */
            class EdgeAccelerationHolonomicGoal : public BaseTebMultiEdge<3, const fakeros::Twist*>
            {
            public:

                /**
                 * @brief Construct edge.
                 */
                EdgeAccelerationHolonomicGoal()
                {
                    _measurement = nullptr;
                    this->resize(3);
                }

                /**
                 * @brief Actual cost function
                 */
                void computeError()
                {
                    if (!m_config || !_measurement)
                    {
                        misc::Logger::getInstance()->error("You must call setTebConfig() and setGoalVelocity() on EdgeAccelerationHolonomicGoal()");
                        assert(false);
                    }

                    const VertexPose* pose_pre_goal = static_cast<const VertexPose*>(_vertices[0]);
                    const VertexPose* pose_goal = static_cast<const VertexPose*>(_vertices[1]);
                    const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);

                    // VELOCITY & ACCELERATION

                    Eigen::Vector2d diff = pose_goal->position() - pose_pre_goal->position();

                    double cos_theta1 = std::cos(pose_pre_goal->theta());
                    double sin_theta1 = std::sin(pose_pre_goal->theta());

                    // transform pose2 into robot frame pose1 (inverse 2d rotation matrix)
                    double p1_dx =  cos_theta1 * diff.x() + sin_theta1 * diff.y();
                    double p1_dy = -sin_theta1 * diff.x() + cos_theta1 * diff.y();

                    double vel1_x = p1_dx / dt->dt();
                    double vel1_y = p1_dy / dt->dt();
                    double vel2_x = _measurement->linear.x;
                    double vel2_y = _measurement->linear.y;

                    double acc_lin_x  = (vel2_x - vel1_x) / dt->dt();
                    double acc_lin_y  = (vel2_y - vel1_y) / dt->dt();

                    _error[0] = penaltyBoundToInterval(acc_lin_x, m_config->robot.acc_lim_x, m_config->optim.penalty_epsilon);
                    _error[1] = penaltyBoundToInterval(acc_lin_y, m_config->robot.acc_lim_y, m_config->optim.penalty_epsilon);

                    // ANGULAR ACCELERATION
                    double omega1 = g2o::normalize_theta(pose_goal->theta() - pose_pre_goal->theta()) / dt->dt();
                    double omega2 = _measurement->angular.z;
                    double acc_rot  = (omega2 - omega1) / dt->dt();

                    _error[2] = penaltyBoundToInterval(acc_rot, m_config->robot.acc_lim_theta, m_config->optim.penalty_epsilon);

                    if (!std::isfinite(_error[0]))
                    {
                        misc::Logger::getInstance()->error(fmt::format("EdgeAccelerationStart::computeError() translational: _error[0]={}\n", _error[0]));
                        assert(false);
                    }
                    if (!std::isfinite(_error[1]))
                    {
                        misc::Logger::getInstance()->error(fmt::format("EdgeAccelerationStart::computeError() strafing: _error[1]={}\n", _error[1]));
                        assert(false);
                    }
                    if (!std::isfinite(_error[2]))
                    {
                        misc::Logger::getInstance()->error(fmt::format("EdgeAccelerationStart::computeError() rotational: _error[2]={}\n", _error[2]));
                        assert(false);
                    }
                }


                /**
                 * @brief Set the goal / final velocity that is taken into account for calculating the acceleration
                 * @param vel_goal twist message containing the translational and rotational velocity
                 */
                void setGoalVelocity(const fakeros::Twist& vel_goal)
                {
                    _measurement = &vel_goal;
                }

                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            };
        }
    }
}

#endif // EDGE_ACCELERATION_HPP
