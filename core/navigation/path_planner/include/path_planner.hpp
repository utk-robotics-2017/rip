#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

#include <vector>

#include <json.hpp>
#include <STP.hh>

#include "point.hpp"
#include "waypoint.hpp"

namespace rip
{
    using Distance = units::Distance;
    using Velocity = units::Velocity;
    using Acceleration = units::Acceleration;
    using Jerk = units::Jerk;
    using Time = units::Time;


    namespace navigation
    {
        template <typename Spline>
        class PathPlanner
        {
        public:
            PathPlanner(const std::vector<Waypoint>& waypoints)
                : m_waypoints(waypoints)
                , m_motion_profile(3)
                , m_spline(waypoints)
            {}

            void setRobotConfig(const Distance& width, const Velocity& max_v, const Acceleration& max_a, const Jerk& max_j)
            {
                m_width = width;
                m_motion_profile.setMax(max_v(), max_a(), max_j());
            }

            void calculate()
            {
                m_motion_profile.planFastestProfile(m_spline.totalLength()(), 0, 0, 0);
            }

            Distance totalDistance()
            {
                return m_spline.totalLength();
            }
            Time totalTime() const
            {
                return m_motion_profile.getDuration();
            }

            Angle heading(const Time& t)
            {
                Distance x = m_motion_profile.pos(t());
                return atan(m_spline.tangent(x));
            }

            Point leftPosition(const Time& t)
            {
                Point center = centerPosition(t);
                Angle h = heading(t);

                return Point(center.x() - m_width / 2.0 * sin(h),
                             center.y() + m_width / 2.0 * cos(h));
            }

            Point centerPosition(const Time& t)
            {
                return m_spline.position(m_motion_profile.pos(t()));
            }

            Point rightPosition(const Time& t)
            {
                Point center = centerPosition(t);
                Angle h = heading(t);

                return Point(center.x() + m_width / 2.0 * sin(h),
                             center.y() - m_width / 2.0 * cos(h));
            }

            Distance centerDistance(const Time& t)
            {
                return m_motion_profile.pos(t());
            }

            Velocity centerVelocity(const Time& t)
            {
                return m_motion_profile.vel(t());
            }

            Acceleration centerAcceleration(const Time& t)
            {
                return m_motion_profile.acc(t());
            }

            Jerk centerJerk(const Time& t)
            {
                return m_motion_profile.jerk(t());
            }

            void writePlan(const std::string& file, const Time& dt)
            {
                nlohmann::json path;

                Time t = 0;

                nlohmann::json prev_segment;
                prev_segment["time"] = 0;
                prev_segment["heading"] = heading(0);

                prev_segment["left"]["position"] = leftPosition(0);
                prev_segment["left"]["distance"] = 0;
                prev_segment["left"]["velocity"] = 0;
                prev_segment["left"]["acceleration"] = 0;
                prev_segment["left"]["jerk"] = 0;

                prev_segment["right"]["position"] = rightPosition(0);
                prev_segment["right"]["distance"] = 0;
                prev_segment["right"]["velocity"] = 0;
                prev_segment["right"]["acceleration"] = 0;
                prev_segment["right"]["jerk"] = 0;

                path.push_back(prev_segment);

                while (t <= m_spline.totalTime())
                {
                    t += dt;

                    nlohmann::json segment;
                    segment["time"] = t;
                    segment["heading"] = heading(t);

                    segment["left"]["position"] = leftPosition(t);
                    segment["left"]["distance"] = prev_segment["left"]["distance"].get<Distance>() + segment["left"]["position"].get<Point>().distance(prev_segment["left"]["position"].get<Point>());
                    segment["left"]["velocity"] = prev_segment["left"]["velocity"].get<Velocity>() + (segment["left"]["distance"].get<Distance>() - prev_segment["left"]["distance"].get<Distance>()) / dt;
                    segment["left"]["acceleration"] = prev_segment["left"]["acceleration"].get<Acceleration>() + (segment["left"]["velocity"].get<Velocity>() - prev_segment["left"]["velocity"].get<Velocity>()) / dt;
                    segment["left"]["jerk"] = prev_segment["left"]["acceleration"].get<Acceleration>() + (segment["left"]["velocity"].get<Velocity>() - prev_segment["left"]["velocity"].get<Velocity>()) / dt;

                    segment["right"]["position"] = rightPosition(t);
                    segment["right"]["distance"] = prev_segment["right"]["distance"].get<Distance>() + segment["right"]["position"].get<Point>().distance(prev_segment["right"]["position"].get<Point>());
                    segment["right"]["velocity"] = prev_segment["right"]["velocity"].get<Velocity>() + (segment["right"]["distance"].get<Distance>() - prev_segment["right"]["distance"].get<Distance>()) / dt;
                    segment["right"]["acceleration"] = prev_segment["right"]["acceleration"].get<Acceleration>() + (segment["right"]["velocity"].get<Velocity>() - prev_segment["right"]["velocity"].get<Velocity>()) / dt;
                    segment["right"]["jerk"] = prev_segment["right"]["acceleration"].get<Acceleration>() + (segment["right"]["velocity"].get<Velocity>() - prev_segment["right"]["velocity"].get<Velocity>()) / dt;

                    path.push_back(segment);

                    prev_segment = segment;
                }

                // write to file
                path;
            }

        private:
            std::vector<Waypoint> m_waypoints;

            stp::STP m_motion_profile;
            Spline m_spline;

            Distance m_width;
        };
    }
}

#endif // PATH_PLANNER_HPP
