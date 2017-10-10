#ifndef AHRS_POS_TIMESTAMP_UPDATE_HPP
#define AHRS_POS_TIMESTAMP_UPDATE_HPP

#include "ahrs_pos_update.hpp"

namespace rip
{
    namespace navigation
    {
        namespace navx
        {
            class AhrsPosTimestampUpdate : public AhrsPosUpdate
            {
            public:
                uint32_t timestamp;

                /**
                 * Default Constructor
                 */
                AhrsPosTimestampUpdate() = default;

                /**
                 * Constructor that decodes the message buffer
                 */
                AhrsPosTimestampUpdate(const std::string& buffer);

                /**
                 * Encodes the object into a message buffer to send
                 * @returns The message buffer
                 */
                std::string encode();
            private:
                // AHRSAndPositioningWithTimestamp (similar to AhrsPos, but adds sample timestamp */
                struct AhrsAndPositioningWithTimestamp
                {
                    static constexpr char kMsgId                  = 't';
                    static constexpr uint8_t kYawIndex            = 4;  // Signed 16:16.  Signed Hundredths
                    static constexpr uint8_t kRollIndex           = 8;  // Signed 16:16.  Signed Hundredths
                    static constexpr uint8_t kPitchIndex          = 12; // Signed 16:16.  Signed Hundredths
                    static constexpr uint8_t kHeadingIndex        = 16; // Signed 16:16.  Unsigned Hundredths
                    static constexpr uint8_t kAltitudeIndex       = 20; // Meters.   Signed 16:16
                    static constexpr uint8_t kFusedHeadingIndex   = 24; // Degrees.  Unsigned Hundredths
                    static constexpr uint8_t kLinearAccelXIndex   = 28; // Inst. G.  Signed Thousandths
                    static constexpr uint8_t kLinearAccelYIndex   = 32; // Inst. G.  Signed Thousandths
                    static constexpr uint8_t kLinearAccelZIndex   = 36; // Inst. G.  Signed Thousandths
                    static constexpr uint8_t kVelocityXIndex      = 40; // Signed 16:16, in meters/sec
                    static constexpr uint8_t kVelocityYIndex      = 44; // Signed 16:16, in meters/sec
                    static constexpr uint8_t kVelocityZIndex      = 48; // Signed 16:16, in meters/sec
                    static constexpr uint8_t kDisplacementXIndex  = 52; // Signed 16:16, in meters
                    static constexpr uint8_t kDisplacementYIndex  = 56; // Signed 16:16, in meters
                    static constexpr uint8_t kDisplacementZIndex  = 60; // Signed 16:16, in meters
                    static constexpr uint8_t kQuatWIndex          = 64; // INT16
                    static constexpr uint8_t kQuatXIndex          = 68; // INT16
                    static constexpr uint8_t kQuatYIndex          = 72; // INT16
                    static constexpr uint8_t kQuatZIndex          = 76; // INT16
                    static constexpr uint8_t kMpuTempValueIndex   = 80; // Centigrade.  Signed Hundredths
                    static constexpr uint8_t kOpstatusIndex       = 82; // NAVX_OP_STATUS_XXX
                    static constexpr uint8_t kSensorStatusIndex   = 83; // NAVX_SENSOR_STATUS_XXX
                    static constexpr uint8_t kCalStatusIndex      = 84; // NAVX_CAL_STATUS_XXX
                    static constexpr uint8_t kSelftestStatusIndex = 85; // NAVX_SELFTEST_STATUS_XXX
                    static constexpr uint8_t kTimestampIndex      = 86; // uint32 Timestamp in milliseconds
                    static constexpr uint8_t kChecksumIndex       = 90;
                    static constexpr uint8_t kTerminatorIndex     = 92;
                    static constexpr uint8_t kMessageLength       = 94;
                };
            };
        } // navx
    } // navigation
} // rip

#endif // AHRS_POS_TIMESTAMP_UPDATE_HPP