#include "pathfinder/profile/scurve.hpp"

#include "pathfinder/segment.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfinder
        {
            std::shared_ptr<Segment> SCurve::calculateSingle(std::shared_ptr<Segment> previous, const Time& time, Status& status)
            {
                std::shared_ptr<Segment> zero_segment = std::make_shared<Segment>();
                zero_segment->time = 0;
                zero_segment->distance = 0;
                zero_segment->velocity = 0;
                zero_segment->acceleration = 0;
                status = Status::kLevel;

                if (!previous)
                {
                    previous = zero_segment;
                }

                std::shared_ptr<Segment> out = std::make_shared<Segment>();
                out->time = time;

                Time dt = time - previous->time;
                if (fabs(previous->distance() - m_setpoint) <= m_tolerance)
                {
                    // Drop all to 0 and return
                    out->acceleration = 0;
                    out->velocity = 0;
                    out->distance = previous->distance;
                    status = Status::kDone;
                    return out;
                }

                Jerk jerk = 0;
                if (previous->distance() < m_setpoint)
                {
                    jerk = m_jerk;
                }
                else
                {
                    jerk = -m_jerk;
                }

                Acceleration max_acceleration = (m_setpoint > 0 ? m_max_acceleration : -m_max_acceleration);
                Time triangle_peak_time = sqrt(2 * previous->velocity() / 2 / jerk());
                Time saturation_time = (-max_acceleration - previous->acceleration) / -jerk;

                Distance dejerk_distance = previous->velocity * triangle_peak_time;
                if (fabs(saturation_time()) < fabs(triangle_peak_time()))
                {
                    Time t0 = (-max_acceleration - previous->acceleration) / -jerk;
                    Time t2 = max_acceleration / jerk;

                    Acceleration a0 = previous->acceleration;
                    Velocity v0 = previous->velocity;
                    float sixth = 1 / 6.0;

                    Velocity v1 = v0 + a0 * t0 + 0.5 * -jerk * t0 * t0;
                    Velocity v2 = max_acceleration * t2 + 0.5 * -jerk * t2 * t2;

                    if (v1 < v2)
                    {
                        t0 = 0;
                    }

                    Time t1 = (v1 - v2) / max_acceleration;
                    if ( t1() < 0)
                    {
                        t2 = t2 + t1;
                        t1 = 0;
                    }

                    Distance s0 = v0 * t0 + 0.5 * a0 * t0 * t0 + sixth * (-jerk) * t0 * t0 * t0;
                    Distance s1 = v1 * t1 + 0.5 * (-max_acceleration) * t1 * t1;
                    Distance s2 = v2 * t2 + 0.5 * (-max_acceleration) * t2 * t2 + sixth * jerk * t2 * t2 * t2;

                    dejerk_distance = s0 + s1 + s2;
                }

                Distance dejerk_error = previous->distance() + dejerk_distance() - m_setpoint;

                std::shared_ptr<Segment> velocity_in = std::make_shared<Segment>();
                velocity_in->time = previous->time;
                velocity_in->distance = previous->velocity();
                velocity_in->velocity = previous->acceleration();

                if (fabs(dejerk_error()) <= m_tolerance ||
                        m_setpoint < 0 && dejerk_error < -m_tolerance ||
                        m_setpoint > 0 && dejerk_error > m_tolerance)
                {
                    m_velocity_profile->m_distance_interval = previous->distance;
                    m_velocity_profile->setpoint(0);

                    std::shared_ptr<Segment> velocity_segment = m_velocity_profile->calculate(velocity_in, time);
                    m_jerk_out = velocity_segment->acceleration();
                    out->acceleration = velocity_segment->velocity();
                    out->velocity = velocity_segment->distance();
                    out->distance = m_velocity_profile->m_distance_interval;
                    status = Status::kDecel;
                }
                else if (fabs(previous->velocity()) < m_max_velocity() - m_tolerance)
                {
                    m_velocity_profile->m_distance_interval = previous->distance;
                    m_velocity_profile->setpoint(m_setpoint < 0 ? -m_max_velocity() : m_max_velocity());

                    std::shared_ptr<Segment> velocity_segment = m_velocity_profile->calculate(velocity_in, time);
                    m_jerk_out = velocity_segment->acceleration();
                    out->acceleration = velocity_segment->velocity();
                    out->velocity = velocity_segment->distance();
                    out->distance = m_velocity_profile->m_distance_interval;
                    status = Status::kAccel;
                }

                if (status == Status::kLevel)
                {
                    m_jerk_out = 0;
                    out->acceleration = 0;
                    out->velocity = previous->velocity;
                    out->distance = previous->distance + previous->velocity * dt;
                }

                return out;
            }
        }
    }
}