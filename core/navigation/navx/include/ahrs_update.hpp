#ifndef AHRS_UPDATE_HPP
#define AHRS_UPDATE_HPP

#include <string>

#include "ahrs_update_base.hpp"

namespace rip
{
    namespace navigation
    {
        namespace navx
        {
            class AhrsUpdate : public AhrsUpdateBase
            {
                short cal_mag_x;
                short cal_mag_y;
                short cal_mag_z;
                float mag_field_norm_ratio;
                float mag_field_norm_scalar;
                short raw_mag_x;
                short raw_mag_y;
                short raw_mag_z;

                /**
                 * Default Constructor
                 */
                AhrsUpdate() = default;

                /**
                 * Decode a message buffer into this object
                 */
                AhrsUpdate(const std::string& buffer);

                /**
                 * Encode this object into a message buffer to send
                 * @returns The message buffer to send
                 */
                std::string encode();

            private:
                // AHRS Update Packet - e.g., !a[yaw][pitch][roll][heading][altitude][fusedheading][accelx/y/z][angular rot x/y/z][opstatus][fusionstatus][cr][lf]
                struct AhrsUpdatePacket
                {
                    static constexpr char    kMsgId                = 'a';
                    static constexpr uint8_t kYawIndex             = 4;  // Degrees. Signed Hundredths
                    static constexpr uint8_t kRollIndex            = 6;  // Degrees. Signed Hundredths
                    static constexpr uint8_t kPitchIndex           = 8;  // Degrees. Signed Hundredths
                    static constexpr uint8_t kHeadingIndex         = 10; // Degrees. Unsigned Hundredths
                    static constexpr uint8_t kAltitudeIndex        = 12; // Meters. Signed 16:16
                    static constexpr uint8_t kFusedHeadingIndex    = 16; // Degrees. Unsigned Hundredths
                    static constexpr uint8_t kLinearAccelXIndex    = 18; // Inst. G. Signed Thousandths
                    static constexpr uint8_t kLinearAccelYIndex    = 20; // Inst. G. Signed Thousandths
                    static constexpr uint8_t kLinearAccelZIndex    = 22; // Inst. G. Signed Thousandths
                    static constexpr uint8_t kCalMagXIndex         = 24; // Int16 (Device Units)
                    static constexpr uint8_t kCalMagYIndex         = 26; // Int16 (Device Units)
                    static constexpr uint8_t kCalMagZIndex         = 28; // Int16 (Device Units)
                    static constexpr uint8_t kCalMagNormRatioIndex = 30; // Ratio. Unsigned Hundredths
                    static constexpr uint8_t kCalMagScalarIndex    = 32; // Coefficient. Signed 16:16
                    static constexpr uint8_t kMpuTempIndex         = 36; // Centigrade. Signed Hundredths
                    static constexpr uint8_t kRawMagXIndex         = 38; // Int16 (Device Units)
                    static constexpr uint8_t kRawMagYIndex         = 40; // Int16 (Device Units)
                    static constexpr uint8_t kRawMagZIndex         = 42; // Int16 (Device Units)
                    static constexpr uint8_t kQuatWIndex           = 44; // Int16
                    static constexpr uint8_t kQuatXIndex           = 46; // Int16
                    static constexpr uint8_t kQuatYIndex           = 48; // Int16
                    static constexpr uint8_t kQuatZIndex           = 50; // Int16
                    static constexpr uint8_t kBaroPressureIndex    = 52; // millibar. Signed 16:16
                    static constexpr uint8_t kBaroTempIndex        = 56; // Centigrade
                    static constexpr uint8_t kOpstatusIndex        = 58; // NAVX_OP_STATUS_XXX
                    static constexpr uint8_t kSensorStatusIndex    = 59; // NAVX_SENSOR_STATUS_XXX
                    static constexpr uint8_t kCalStatusIndex       = 60; // NAVX_CAL_STATUS_XXX
                    static constexpr uint8_t kSelftestStatusIndex  = 61; // NAVX_SELFTEST_STATUS_XXX
                    static constexpr uint8_t kChecksumIndex        = 62;
                    static constexpr uint8_t kTerminatorIndex      = 64;
                    static constexpr uint8_t kMessageLength        = 66;
                };
            };
        } // navx
    } // navigation
} // rip

#endif // AHRS_UPDATE_HPP