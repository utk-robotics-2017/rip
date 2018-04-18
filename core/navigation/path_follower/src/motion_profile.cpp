#include "path_follower/motion_profile.hpp"

#include "path_follower/epsilon.hpp"

#include <misc/logger.hpp>


namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            MotionProfile::MotionProfile()
                    : m_segments()
            {
            }

            MotionProfile::MotionProfile(const std::vector<MotionSegment>& segments)
                    : m_segments(segments)
            {
            }

            bool MotionProfile::valid() const
            {
                MotionSegment previous = m_segments.front();
                if(!previous.valid())
                {
                    return false;
                }
                for(int i = 1, end = m_segments.size(); i < end; i++)
                {
                    MotionSegment segment = m_segments[i];
                    if(!segment.valid())
                    {
                        return false;
                    }

                    if(!segment.start().coincident(previous.end()))
                    {
                        return false;
                    }
                    previous = segment;
                }
            }

            bool MotionProfile::empty() const
            {
                return m_segments.empty();
            }

            nonstd::optional<MotionState> MotionProfile::stateByTime(const units::Time& t)
            {
                if(t < startTime() && t + units::Time(k_epsilon) >= startTime())
                {
                    return nonstd::make_optional(startState());
                }

                if(t > endTime() && t + units::Time(k_epsilon) <= endTime())
                {
                    return nonstd::make_optional(endState());
                }

                for(const MotionSegment& s : m_segments)
                {
                    if(s.containsTime(t))
                    {
                        return nonstd::make_optional(s.start().extrapolate(t));
                    }
                }

                return nonstd::nullopt;
            }

            MotionState MotionProfile::stateByTimeClamped(const units::Time& t)
            {
                if(t < startTime())
                {
                    return startState();
                }else if(t > endTime())
                {
                    return endState();
                }
                for(const MotionSegment& segment: m_segments)
                {
                    if(segment.containsTime(t))
                    {
                        return segment.start().extrapolate(t);
                    }
                }

                return MotionState::k_invalid_state;
            }

            nonstd::optional<MotionState> MotionProfile::firstStateByPosition(const units::Distance& pos) const
            {
                for(const MotionSegment& segment : m_segments)
                {
                    if(segment.containsPosition(pos))
                    {
                        if(epsilonEquals<units::Distance>(segment.end().position(), pos, k_epsilon))
                        {
                            return nonstd::make_optional(segment.end());
                        }
                        const units::Time t = units::min(segment.start().nextTimeAtPosition(pos), segment.end().t());
                        if(units::isnan(t))
                        {
                            misc::Logger::getInstance()->error("Error! We should reach 'pos' but don't");
                            return nonstd::nullopt;
                        }
                        return nonstd::make_optional(segment.start().extrapolate(t));
                    }
                }

                // pos is never reached
                return nonstd::nullopt;
            }

            void MotionProfile::trimBeforeTime(const units::Time& t)
            {
                for(size_t i = 0; i < m_segments.size(); i++)
                {
                    MotionSegment& segment = m_segments[i];
                    if(segment.end().t() <= t)
                    {
                        m_segments.erase(m_segments.begin() + i);
                        i--;
                        continue;
                    }

                    if(segment.start().t() <= t)
                    {
                        segment.setStart(segment.start().extrapolate(t));
                    }
                    break;
                }
            }

            void MotionProfile::clear()
            {
                m_segments.clear();
            }

            void MotionProfile::reset(const MotionState& initial_state)
            {
                clear();
                m_segments.emplace_back(initial_state, initial_state);
            }

            void MotionProfile::consolidate()
            {
                for(size_t i = 0; i < m_segments.size(); i++)
                {
                    MotionSegment& segment = m_segments[i];
                    if(segment.start().coincident(segment.end()))
                    {
                        m_segments.erase(m_segments.begin() + i);
                        i--;
                    }
                }
            }

            void MotionProfile::appendControl(const units::Acceleration& acc, const units::Time& dt)
            {
                if(empty())
                {
                    misc::Logger::getInstance()->error("Error! Trying to append to an empty profile");
                    return;
                }
                MotionState last_end_state = m_segments.back().end();
                MotionState new_start_state(last_end_state.t(), last_end_state.position(), last_end_state.velocity(), acc);
                m_segments.emplace_back(new_start_state, new_start_state.extrapolate(new_start_state.t() + dt));
            }

            void MotionProfile::appendSegment(const MotionSegment& segment)
            {
                m_segments.push_back(segment);
            }

            void MotionProfile::appendProfile(const MotionProfile& profile)
            {
                for(const MotionSegment& segment : profile.segments())
                {
                    m_segments.push_back(segment);
                }
            }

            size_t MotionProfile::size() const
            {
                return m_segments.size();
            }

            std::vector<MotionSegment> MotionProfile::segments() const
            {
                return m_segments;
            }

            MotionState MotionProfile::startState() const
            {
                if(empty())
                {
                    return MotionState::k_invalid_state;
                }
                return m_segments.front().start();
            }

            units::Time MotionProfile::startTime() const
            {
                return startState().t();
            }

            units::Distance MotionProfile::startPosition() const
            {
                return startState().position();
            }

            MotionState MotionProfile::endState() const
            {
                if(empty())
                {
                    return MotionState::k_invalid_state;
                }
                return m_segments.back().start();
            }

            units::Time MotionProfile::endTime() const
            {
                return endState().t();
            }

            units::Distance MotionProfile::endPosition() const
            {
                return endState().position();
            }

            units::Time MotionProfile::duration() const
            {
                return endTime() - startTime();
            }

            units::Distance MotionProfile::length() const
            {
                units::Distance rv = 0.0;
                for(const MotionSegment& segment : m_segments)
                {
                    rv += units::abs(segment.end().position() - segment.start().position());
                }
                return rv;
            }

            std::string MotionProfile::toString() const
            {
                std::string rv = "Profile:";
                for(const MotionSegment& segment : m_segments)
                {
                    rv += "\n\t";
                    rv += segment.toString();
                }
                return rv;
            }

            std::ostream& operator<<(std::ostream& os, const MotionProfile& profile)
            {
                os << profile.toString() << std::endl;
                return os;
            }
        }
    }
}