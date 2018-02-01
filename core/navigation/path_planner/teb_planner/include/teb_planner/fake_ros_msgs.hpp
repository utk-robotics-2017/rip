#ifndef FAKE_ROS_MSGS_HPP
#define FAKE_ROS_MSGS_HPP

namespace rip
{
    namespace navigation
    {
        namespace tebplanner
        {
            namespace fakeros
            {
                struct Vector3d
                {
                public:
                    double x;
                    double y;
                    double z;
                };

                using Point = Vector3d;

                struct Twist
                {
                    Vector3d linear;
                    Vector3d angular;
                };

                struct TwistWithCovariance
                {
                    Twist twist;
                    double covariance[36];
                };

                struct Time
                {
                    unsigned int sec;
                    unsigned int nsec;
                };

                struct Duration
                {
                    int sec;
                    int nsec;
                };

                struct Header
                {
                    int seq;
                    int stamp;
                };

                struct Quaternion
                {
                    double x;
                    double y;
                    double z;
                    double w;
                };

                struct QuaternionStamped
                {
                    Header header;
                    Quaternion quaternion;
                };

                struct Pose
                {
                    Vector3d position;
                    Quaternion orientation;
                };

                struct PoseStamped
                {
                    Header header;
                    Pose pose;
                };

                struct TrajectoryPointMsg
                {
                    Pose pose;
                    Twist velocity;
                    Twist Acceleration;
                    Duration time_from_start;
                };

                struct TrajectoryMsg
                {
                    Header header;
                    TrajectoryPointMsg trajectory[];
                };
            }
        }
    }
}

#endif // FAKE_ROS_MSGS_HPP
