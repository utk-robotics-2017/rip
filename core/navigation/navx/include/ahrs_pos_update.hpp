#ifndef AHRS_POS_UPDATE_HPP
#define AHRS_POS_UPDATE_HPP

#include <string>

#include "ahrs_update_base.hpp"

namespace rip
{
    namespace navigation
    {
        namespace navx
        {
            class AhrsPosUpdate : public AhrsUpdateBase
            {
            public:
                float velocity_x;
                float velocity_y;
                float velocity_z;
                float displacement_x;
                float displacement_y;
                float displacement_z;

                /**
                 * Default Constructor
                 */
                AhrsPosUpdate() = default;

                /**
                 * Constructor that decodes a message buffer
                 * @param buffer
                 */
                AhrsPosUpdate(const std::string& buffer);

                /**
                 * Encodes this object into a message buffer to send
                 * @returns The message buffer
                 */
                std::string encode();

            private:
                // AHRSAndPositioning Update Packet (similar to AHRS, but removes magnetometer and adds velocity/displacement) */
                struct AhrsAndPositioningUpdatePacket
                {
                    static constexpr char kMsgId                  = 'p';
                    static constexpr uint8_t kYawIndex            = 4;  // Degrees.  Signed Hundredths
                    static constexpr uint8_t kRollIndex           = 6;  // Degrees.  Signed Hundredths
                    static constexpr uint8_t kPitchIndex          = 8;  // Degrees.  Signed Hundredths
                    static constexpr uint8_t kHeadingIndex        = 10; // Degrees.  Unsigned Hundredths
                    static constexpr uint8_t kAltitudeIndex       = 12; // Meters.   Signed 16:16
                    static constexpr uint8_t kFusedHeadingIndex   = 16; // Degrees.  Unsigned Hundredths
                    static constexpr uint8_t kLinearAccelXIndex   = 18; // Inst. G.  Signed Thousandths
                    static constexpr uint8_t kLinearAccelYIndex   = 20; // Inst. G.  Signed Thousandths
                    static constexpr uint8_t kLinearAccelZIndex   = 22; // Inst. G.  Signed Thousandths
                    static constexpr uint8_t kVelocityXIndex      = 24; // Signed 16:16, in meters/sec
                    static constexpr uint8_t kVelocityYIndex      = 28; // Signed 16:16, in meters/sec
                    static constexpr uint8_t kVelocityZIndex      = 32; // Signed 16:16, in meters/sec
                    static constexpr uint8_t kDisplacementXIndex  = 36; // Signed 16:16, in meters
                    static constexpr uint8_t kDisplacementYIndex  = 40; // Signed 16:16, in meters
                    static constexpr uint8_t kDisplacementZIndex  = 44; // Signed 16:16, in meters
                    static constexpr uint8_t kQuatWIndex          = 48; // INT16
                    static constexpr uint8_t kQuatXIndex          = 50; // INT16
                    static constexpr uint8_t kQuatYIndex          = 52; // INT16
                    static constexpr uint8_t kQuatZIndex          = 54; // INT16
                    static constexpr uint8_t kMpuTempValueIndex   = 56; // Centigrade.  Signed Hundredths
                    static constexpr uint8_t kOpstatusIndex       = 58; // NAVX_OP_STATUS_XXX
                    static constexpr uint8_t kSensorStatusIndex   = 59; // NAVX_SENSOR_STATUS_XXX
                    static constexpr uint8_t kCalStatusIndex      = 60; // NAVX_CAL_STATUS_XXX
                    static constexpr uint8_t kSelftestStatusIndex = 61; // NAVX_SELFTEST_STATUS_XXX
                    static constexpr uint8_t kChecksumIndex       = 62;
                    static constexpr uint8_t kTerminatorIndex     = 64;
                    static constexpr uint8_t kMessageLength       = 66;
                };
            };
        } // navx
    } // navigation
} // rip

#endif // AHRS_POS_UPDATE_HPP