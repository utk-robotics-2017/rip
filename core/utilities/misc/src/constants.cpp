#include <misc/constants.hpp>


namespace rip
{
    namespace misc
    {
        const char* constants::kArduinoGenHome = "arduino_gen_home";
        const char* constants::kLoggerName = "rip_logger";
        const char* constants::kMaxVelocity = "max_velocity";
        const char* constants::kMaxAcceleration = "max_acceleration";
        const char* constants::kSegmentCompletionTolerance = "segment_completion_tolerance";
        const char* constants::kWheelbase = "wheelbase";
        const char* constants::kTrackScrubFactor = "scrub_factor";
        const char* constants::kTurnKp = "turn_kp";
        const char* constants::kTurnKi = "turn_ki";
        const char* constants::kTurnKd = "turn_kd";
        const char* constants::kStraightAngleKp = "straight_angle_kp";
        const char* constants::kStraightAngleKi = "straight_angle_ki";
        const char* constants::kStraightAngleKd = "straight_angle_kd";
        const char* constants::kStraightThreshTime = "straight_threshold_time";

        std::shared_ptr<constants> constants::m_singleton = nullptr;
    }
}
