#ifndef PROFILE_HPP
#define PROFILE_HPP

#include <memory>

#include <units.hpp>

namespace rip
{
    namespace navigation
    {
        namespace pathfinder
        {
            class Segment;

            /**
             * Base class for motion profiles
             */
            class Profile
            {
            public:
                /**
                 * Constructor
                 */
                Profile();

                /**
                 * Setter for the setpoint
                 */
                void setpoint(float setpoint);

                /**
                 * Getter for the setpoint
                 */
                float setpoint() const;

                /**
                 * Setter for the tolerance
                 */
                void tolerance(float tolerance);

                /**
                 * Getter for the tolerance
                 */
                float tolerance() const;

                /**
                 * Setter for the timescale
                 */
                void timescale(const Time& timescale);

                /**
                 * Getter for the timescale
                 */
                Time timescale() const;

                /**
                 *
                 */
                virtual std::shared_ptr<Segment> calculate(std::shared_ptr<Segment> previous, const Time& time);

            protected:
                enum class Status
                {
                    kAccel,
                    kDecel,
                    kLevel,
                    kDone
                };

                /**
                 *
                 */
                virtual std::shared_ptr<Segment> calculateSingle(std::shared_ptr<Segment> previous, const Time& time, Status& status) = 0;

                static const Time kDefaultTimescale;
                static const float kDefaultTolerance;

                float m_setpoint;
                float m_tolerance;
                Time m_timescale;
            }; // class Profile
        } // namespace pathfinder
    } // namespace navigation
} // namespace rip

#endif // PROFILE_HPP
