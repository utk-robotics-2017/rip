#ifndef MOTION_PROFILE_HPP
#define MOTION_PROFILE_HPP

#include <ostream>
#include <vector>

#include <units/units.hpp>
#include <optional.hpp>

#include "path_follower/motion_segment.hpp"
#include "path_follower/motion_state.hpp"

namespace rip
{
    namespace navigation
    {
        namespace pathfollower
        {
            class MotionProfile
            {
            public:
                /**
                 * Create an empty MotionProfile
                 */
                MotionProfile();

                /**
                 * Create a MotionProfile from an existing list of segments
                 *
                 * @param segments The new segments of the profile
                 */
                MotionProfile(const std::vector<MotionSegment>& segments);

                /**
                 * Checks if the given MotionProfile is valid. This checks that:
                 *
                 * 1. All segments are valid.
                 *
                 * 2. Successive segments are C1 continuous in position and C0 continuous in velocity;
                 *
                 * @return True if the MotionProfile is valid
                 */
                bool valid() const;

                /**
                 * Check if the profile is empty
                 *
                 * @return True if there are no segments
                 */
                bool empty() const;

                /**
                 * Get the interpolated MotionState at any given time
                 *
                 * @param t The time to query
                 * @return Empty if the time is outside the time bounds of the profile or the resulting MotionProfile otherwise
                 */
                nonstd::optional<MotionState> stateByTime(const units::Time& t);

                /**
                 * Get the interpolated MotionState at any given time, clamping to the endpoints if time is out of bounds
                 *
                 * @param t The time to query
                 * @return The resulting MotionProfile
                 */
                MotionState stateByTimeClamped(const units::Time& t);

                /**
                 * Get the interpolated MotionState by distance
                 *
                 * @note That since a profile may reverse, this method only returns the "first" instance of this position
                 *
                 * @param pos The position to query
                 * @return Empty if the profile never crosses the position or if the profile is invalid, or the resulting MotionProfile
                 *          otherwise
                 */
                nonstd::optional<MotionState> firstStateByPosition(const units::Distance& pos) const;

                /**
                 * Remove all parts of the profile prior to the query time. This eliminates whole segments and also shortens
                 * any segments containing t
                 *
                 * @param t The query time
                 */
                void trimBeforeTime(const units::Time& t);

                /**
                 * Remove all segments
                 */
                void clear();

                /**
                 * Remove all segments and initialize to the desired state (actually a segment of length 0 that starts and ends at
                 * initial_state).
                 *
                 * @param initial_state The MotionState to initialize to
                 */
                void reset(const MotionState& initial_state);

                /**
                 * Remove redundant segments (segments whose start and end states are coincident)
                 */
                void consolidate();

                /**
                 * Add to the profile by applying an acceleration control for a given time. This is appended to the previous last
                 * state
                 *
                 * @param acc The acceleration to apply
                 * @param dt The period of time to apply the given acceleration
                 */
                void appendControl(const units::Acceleration& acc, const units::Time& dt);

                /**
                 * Add to the profile by inserting a new segment. No validity checking is done.
                 *
                 * @param segment The segment to add
                 */
                void appendSegment(const MotionSegment& segment);

                /**
                 * Add to the profile by inserting a new profile after the final state. No validity checking is done.
                 *
                 * @param profile The profile to add
                 */
                void appendProfile(const MotionProfile& profile);

                /**
                 * @return The number of segments
                 */
                size_t size() const;

                /**
                 * @return The list of segments
                 */
                std::vector<MotionSegment> segments() const;

                /**
                 * @return The first state in the profile (or kInvalidState if empty)
                 */
                MotionState startState() const;

                /**
                 * @return The time of the first state in the profile (or NaN if empty)
                 */
                units::Time startTime() const;

                /**
                 * @return The position of the first state in the profile (or NaN if empty)
                 */
                units::Distance startPosition() const;

                /**
                 * @return The last state in the profile (or KInvalidState if empty)
                 */
                MotionState endState() const;

                /**
                 * @return The time of the last state in the profile (or NaN if empty)
                 */
                units::Time endTime() const;

                /**
                 * @return The position of the last state in the profile (or NaN if empty)
                 */
                units::Distance endPosition() const;

                /**
                 * @return The duration of the entire profile
                 */
                units::Time duration() const;

                /**
                 * @return The total distance covered by the profile
                 *
                 * @note The distance is the sum of absolute distances of all segments, so a reversing profile
                 *      will count the distance covered in each direction
                 */
                units::Distance length() const;

                std::string toString() const;

                friend std::ostream& operator<<(std::ostream& os, const MotionProfile& profile);

            protected:
                std::vector<MotionSegment> m_segments;
            };

            std::ostream& operator<<(std::ostream& os, const MotionProfile& profile);
        }
    }
}
#endif //MOTION_PROFILE_HPP