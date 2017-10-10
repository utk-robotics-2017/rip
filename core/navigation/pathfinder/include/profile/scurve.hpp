#ifndef SCURVE_HPP
#define SCURVE_HPP

#include "profile.hpp"
#include "trapezoidal.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfinder
        {
            class SCurve : public Profile
            {
                SCurve(const Velocity& max_velocity, const Acceleration& max_acceleration, const Jerk& jerk,  const Time& timescale = kDefaultTimescale, const float& tolerance = kDefaultTolerance);

            protected:
                virtual std::shared_ptr<Segment> calculateSingle(std::shared_ptr<Segment> previous, const Time& time, Status& status) override;

            private:
                Velocity m_max_velocity;
                Acceleration m_max_acceleration;
                Jerk m_jerk;
                Jerk m_jerk_out;

                std::unique_ptr<Trapezoidal> m_velocity_profile;
            };
        } // namespace pathfinder
    } // namespace navigation
} // namespace rip

#endif // SCURVE_HPP