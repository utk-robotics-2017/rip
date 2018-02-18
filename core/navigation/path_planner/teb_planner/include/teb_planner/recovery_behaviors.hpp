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

#ifndef RECOVERY_BEHAVIORS_HPP
#define RECOVERY_BEHAVIORS_HPP

#include <vector>

#include <boost/circular_buffer.hpp>
#include <teb_planner/fake_ros_msgs.hpp>

namespace rip
{
    namespace navigation
    {
        namespace tebplanner
        {
            namespace circularbuffer
            {
                /*
                   T must implement operator=, copy ctor
                */

                template<typename T> class CircBuf {
                  std::vector<T> data;
                  int front;
                  int count;
                public:
                  CircBuf();
                  CircBuf(int);
                  ~CircBuf();

                  void setCapacity(int n)
                  {
                      data.resize(n);
                  }

                  bool empty() const
                  {
                      return count == 0;
                  }

                  bool full() const
                  {
                      return count == data.size();
                  }

                  int capacity() const
                  {
                      return data.size();
                  }

                  int size() const
                  {
                      return count;
                  }

                  void clear()
                  {
                      data.clear();
                  }

                  bool add(const T&);
                  bool remove(T*);

                  T& operator [](int index);
                };

                template<typename T> CircBuf<T>::CircBuf(int sz)
                {
                  if (sz==0)
                  {
                      throw std::invalid_argument("size cannot be zero");
                  }

                  data.resize(sz);
                  front = 0;
                  count = 0;
                }

                // returns true if add was successful, false if the buffer is already full
                template<typename T> bool CircBuf<T>::add(const T &t)
                {
                  if ( full() )
                  {
                    return false;
                  }
                  else
                  {
                    // find index where insert will occur
                    int end = (front + count) % data.size();
                    data[end] = t;
                    count++;
                    return true;
                  }
                }

                // returns true if there is something to remove, false otherwise
                template<typename T> bool CircBuf<T>::remove(T *t)
                {
                  if ( empty() )
                  {
                    return false;
                  }
                  else
                  {
                    *t = data[front];

                    front = front == data.size() ? 0 : front + 1;
                    count--;
                    return true;
                  }
                }

                template<typename T> T& CircBuf<T>::operator [](int index)
                {
                    return data[index];
                }
            }

            /**
             * @class FailureDetector
             * @brief This class implements methods in order to detect if the robot got stucked or is oscillating
             *
             * The StuckDetector analyzes the last N commanded velocities in order to detect whether the robot
             * might got stucked or oscillates between left/right/forward/backwards motions.
             */
            class FailureDetector
            {
            public:

                /**
                 * @brief Default constructor
                 */
                FailureDetector()
                {}

                /**
                 * @brief destructor.
                 */
                ~FailureDetector()
                {}

                /**
                 * @brief Set buffer length (measurement history)
                 * @param length number of measurements to be kept
                 */
                void setBufferLength(int length)
                {
                    m_buffer.setCapacity(length);
                }

                /**
                 * @brief Add a new twist measurement to the internal buffer and compute a new decision
                 * @param twist fakeros::Twist velocity information
                 * @param v_max maximum forward translational velocity
                 * @param v_backwards_max maximum backward translational velocity
                 * @param omega_max maximum angular velocity
                 * @param v_eps Threshold for the average normalized linear velocity in (0,1) that must not be exceded (e.g. 0.1)
                 * @param omega_eps Threshold for the average normalized angular velocity in (0,1) that must not be exceded (e.g. 0.1)
                 */
                void update(const fakeros::Twist& twist, double v_max, double v_backwards_max, double omega_max, double v_eps, double omega_eps);

                /**
                 * @brief Check if the robot got stucked
                 *
                 * This call does not compute the actual decision,
                 * since it is computed within each update() invocation.
                 * @return true if the robot got stucked, false otherwise.
                 */
                bool isOscillating() const;

                /**
                 * @brief Clear the current internal state
                 *
                 * This call also resets the internal buffer
                 */
                void clear();

            protected:

                /** Variables to be monitored */
                struct VelMeasurement
                {
                    double v = 0;
                    double omega = 0;
                };

                /**
                 * @brief Detect if the robot got stucked based on the current buffer content
                 *
                 * Afterwards the status might be checked using gotStucked();
                 * @param v_eps Threshold for the average normalized linear velocity in (0,1) that must not be exceded (e.g. 0.1)
                 * @param omega_eps Threshold for the average normalized angular velocity in (0,1) that must not be exceded (e.g. 0.1)
                 * @return true if the robot got stucked, false otherwise
                 */
                bool detect(double v_eps, double omega_eps);

            private:



                circularbuffer::CircBuf<VelMeasurement> m_buffer; //!< Circular buffer to store the last measurements @see setBufferLength
                bool m_oscillating = false; //!< Current state: true if robot is oscillating

            };


        }
    }
}

#endif /* RECOVERY_BEHAVIORS_H__ */
