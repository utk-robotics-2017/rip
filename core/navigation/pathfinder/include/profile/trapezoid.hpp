#ifndef TRAPEZOIDAL_HPP
#define TRAPEZOIDAL_HPP

#include <memory>

#include <units.hpp>

#include "profile.hpp"
#include "scurve.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfinder
        {
            class Segment;

            /**
             *  A trapezoid motion profile
             */
            class Trapezoidal : public Profile
            {
            public:
                /**
                 *  Constructor
                 *
                 * @param max_velocity The maximum velocity of the robot
                 * @param acceleration The acceleration of the robot
                 * @param target_timescale The scaling of time used internally
                 */
                Trapezoidal(Velocity max_velocity, Acceleration acceleration, Time target_timescale = Profile::kDefaultTimescale, float tolerance = Profile::kDefaultTolerance);


            protected:
                /**
                 *
                 */
                virtual std::shared_ptr<Segment> calculateSingle(std::shared_ptr<Segment> previous, const Time& time, Status& status) override;

            private:
                Velocity m_max_velocity;
                Acceleration m_acceleration;
                Distance m_distance_interval; //!< Used only for SCurve profile

                friend class SCurve;
            };
        } // namespace pathfinder
    } // namespace navigation
} // namespace rip

#endif // TRAPEZOIDAL_HPP
