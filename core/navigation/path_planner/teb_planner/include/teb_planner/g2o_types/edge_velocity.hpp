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

#ifndef EDGE_VELOCITY_HPP
#define EDGE_VELOCITY_HPP

#include <misc/logger.hpp>

#include <teb_planner/g2o_types/vertex_pose.hpp>
#include <teb_planner/g2o_types/vertex_time_diff.hpp>
#include <teb_planner/g2o_types/base_teb_edges.hpp>
#include <teb_planner/g2o_types/penalties.hpp>
#include <teb_planner/teb_config.hpp>

namespace rip
{
    namespace navigation
    {
        namespace tebplanner
        {


            /**
             * @class EdgeVelocity
             * @brief Edge defining the cost function for limiting the translational and rotational velocity.
             *
             * The edge depends on three vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \Delta T_i \f$ and minimizes: \n
             * \f$ \min \textrm{penaltyInterval}( [v,omega]^T ) \cdot weight \f$. \n
             * \e v is calculated using the difference quotient and the position parts of both poses. \n
             * \e omega is calculated using the difference quotient of both yaw angles followed by a normalization to [-pi, pi]. \n
             * \e weight can be set using setInformation(). \n
             * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval(). \n
             * The dimension of the error / cost vector is 2: the first component represents the translational velocity and
             * the second one the rotational velocity.
             * @see TebOptimalPlanner::AddEdgesVelocity
             * @remarks Do not forget to call setTebConfig()
             */
            class EdgeVelocity : public BaseTebMultiEdge<2, double>
            {
            public:

                /**
                 * @brief Construct edge.
                 */
                EdgeVelocity()
                {
                    this->resize(3); // Since we derive from a g2o::BaseMultiEdge, set the desired number of vertices
                }

                /**
                 * @brief Actual cost function
                 */
                void computeError()
                {
                    if (!m_config)
                    {
                        misc::Logger::getInstance()->error("You must call setTebConfig on EdgeVelocity()");
                        assert(false);
                    }
                    const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
                    const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);
                    const VertexTimeDiff* deltaT = static_cast<const VertexTimeDiff*>(_vertices[2]);

                    const Eigen::Vector2d deltaS = conf2->estimate().position() - conf1->estimate().position();

                    double dist = deltaS.norm();
                    const double angle_diff = g2o::normalize_theta(conf2->theta() - conf1->theta());
                    if (m_config->trajectory.exact_arc_length && angle_diff != 0)
                    {
                        double radius =  dist / (2 * sin(angle_diff / 2));
                        dist = fabs( angle_diff * radius ); // actual arg length!
                    }
                    double vel = dist / deltaT->estimate();

                    vel *= fast_sigmoid( 100 * (deltaS.x() * cos(conf1->theta()) + deltaS.y() * sin(conf1->theta())) ); // consider direction

                    const double omega = angle_diff / deltaT->estimate();

                    _error[0] = penaltyBoundToInterval(vel, -m_config->robot.max_vel_x_backwards, m_config->robot.max_vel_x, m_config->optim.penalty_epsilon);
                    _error[1] = penaltyBoundToInterval(omega, m_config->robot.max_vel_theta, m_config->optim.penalty_epsilon);

                    if (!std::isfinite(_error[0]) || !std::isfinite(_error[1]))
                    {
                        misc::Logger::getInstance()->error(fmt::format("EdgeVelocity::computeError() _error[0]={} _error[1]={}\n", _error[0], _error[1]));
                        assert(false);
                    }
                }

                EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            };


            /**
             * @class EdgeVelocityHolonomic
             * @brief Edge defining the cost function for limiting the translational and rotational velocity according to x,y and theta.
             *
             * The edge depends on three vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \Delta T_i \f$ and minimizes: \n
             * \f$ \min \textrm{penaltyInterval}( [vx,vy,omega]^T ) \cdot weight \f$. \n
             * \e vx denotes the translational velocity w.r.t. x-axis (computed using finite differneces). \n
             * \e vy denotes the translational velocity w.r.t. y-axis (computed using finite differneces). \n
             * \e omega is calculated using the difference quotient of both yaw angles followed by a normalization to [-pi, pi]. \n
             * \e weight can be set using setInformation(). \n
             * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval(). \n
             * The dimension of the error / cost vector is 3: the first component represents the translational velocity w.r.t. x-axis,
             * the second one w.r.t. the y-axis and the third one the rotational velocity.
             * @see TebOptimalPlanner::AddEdgesVelocity
             * @remarks Do not forget to call setTebConfig()
             */
            class EdgeVelocityHolonomic : public BaseTebMultiEdge<3, double>
            {
            public:

                /**
                 * @brief Construct edge.
                 */
                EdgeVelocityHolonomic()
                {
                    this->resize(3); // Since we derive from a g2o::BaseMultiEdge, set the desired number of vertices
                }

                /**
                 * @brief Actual cost function
                 */
                void computeError()
                {
                    if (!m_config)
                    {
                        misc::Logger::getInstance()->error("You must call setTebConfig on EdgeVelocityHolonomic()");
                        assert(false);
                    }

                    const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
                    const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);
                    const VertexTimeDiff* deltaT = static_cast<const VertexTimeDiff*>(_vertices[2]);
                    Eigen::Vector2d deltaS = conf2->position() - conf1->position();

                    double cos_theta1 = std::cos(conf1->theta());
                    double sin_theta1 = std::sin(conf1->theta());

                    // transform conf2 into current robot frame conf1 (inverse 2d rotation matrix)
                    double r_dx =  cos_theta1 * deltaS.x() + sin_theta1 * deltaS.y();
                    double r_dy = -sin_theta1 * deltaS.x() + cos_theta1 * deltaS.y();

                    double vx = r_dx / deltaT->estimate();
                    double vy = r_dy / deltaT->estimate();
                    double omega = g2o::normalize_theta(conf2->theta() - conf1->theta()) / deltaT->estimate();

                    _error[0] = penaltyBoundToInterval(vx, -m_config->robot.max_vel_x_backwards, m_config->robot.max_vel_x, m_config->optim.penalty_epsilon);
                    _error[1] = penaltyBoundToInterval(vy, m_config->robot.max_vel_y, 0.0); // we do not apply the penalty epsilon here, since the velocity could be close to zero
                    _error[2] = penaltyBoundToInterval(omega, m_config->robot.max_vel_theta, m_config->optim.penalty_epsilon);

                    if (!std::isfinite(_error[0]) || !std::isfinite(_error[1]) || !std::isfinite(_error[2]))
                    {
                        misc::Logger::getInstance()->error(fmt::format("EdgeVelocityHolonomic::computeError() _error[0]={} _error[1]={} _error[2]={}\n", _error[0], _error[1], _error[2]));
                        assert(false);
                    }
                }

                EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            };
        }
    }

}

#endif // EDGE_VELOCITY_HPP
