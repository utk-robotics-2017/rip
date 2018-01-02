#ifndef AHRS_PROTOCOL_HPP
constexpr uint8_t AHRS_PROTOCOL_HPP
/* ============================================
navX MXP source code is placed under the MIT license
Copyright (c) 2015 Kauai Labs

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/


namespace rip
{
    namespace navigation
    {
        namespace navx
        {
            /*****************************************************************************
             * This protocol, introduced first with the navX MXP, expands upon the IMU   *
             * protocol by adding the following new functionality:                       *
             *                                                                           *
             * AHRS Update:  Includes Fused Heading and Altitude Info                    *
             * Magnetometer Calibration:  Enables configuration of coefficients from PC  *
             * Board Identity:  Enables retrieval of Board Identification Info           *
             * Fusion Tuning:  Enables configuration of key thresholds/coefficients used *
             *                 in data fusion algorithms from a remote client            *
             *                                                                           *
             * In addition, the navX enable stream command has been extended with a new  *
             * Stream type, in order to enable AHRS Updates.                             *
             *****************************************************************************/

            constexpr char kBinaryPacketIndicatorChar = '#';

            /* AHRS Protocol encodes certain data in binary format, unlike the IMU  *
             * protocol, which encodes all data in ASCII characters.  Thus, the     *
             * packet start and message termination sequences may occur within the  *
             * message content itself.  To support the binary format, the binary    *
             * message has this format:                                             *
             *                                                                      *
             * [start][binary indicator][len][msgid]<MESSAGE>[checksum][terminator] *
             *                                                                      *
             * (The binary indicator and len are not present in the ASCII protocol) *
             *                                                                      *
             * The [len] does not include the length of the start and binary        *
             * indicator characters, but does include all other message items,      *
             * including the checksum and terminator sequence.                      */

            enum class TuningVarId
            {
                kUnspecified             = 0,
                kMotionThreshold         = 1, // G
                kYawStableThreshold      = 2, // Degrees
                kMagDisturbanceThreshold = 3, // Ratio
                kSeaLevelPressure        = 4, // Millibars
                kMinTuningVarId          = kMotionThreshold,
                kMaxTuningVarId          = kSeaLevelPressure
            };

            enum class DataAction
            {
                kDataGet          = 0,
                kDataSet          = 1,
                kDataSetToDefault = 2
            };

            constexpr uint8_t kDataGetSetSuccess = 0;
            constexpr uint8_t kDataGetSetError   = 1;

            // AHRS Update Packet - e.g., !a[yaw][pitch][roll][heading][altitude][fusedheading][accelx/y/z][angular rot x/y/z][opstatus][fusionstatus][cr][lf]
            namespace AhrsUpdatePacket
            {
                constexpr char    kMsgId                = 'a';
                constexpr uint8_t kYawIndex             = 4;  // Degrees. Signed Hundredths
                constexpr uint8_t kRollIndex            = 6;  // Degrees. Signed Hundredths
                constexpr uint8_t kPitchIndex           = 8;  // Degrees. Signed Hundredths
                constexpr uint8_t kHeadingIndex         = 10; // Degrees. Unsigned Hundredths
                constexpr uint8_t kAltitudeIndex        = 12; // Meters. Signed 16:16
                constexpr uint8_t kFusedHeadingIndex    = 16; // Degrees. Unsigned Hundredths
                constexpr uint8_t kLinearAccelXIndex    = 18; // Inst. G. Signed Thousandths
                constexpr uint8_t kLinearAccelYIndex    = 20; // Inst. G. Signed Thousandths
                constexpr uint8_t kLinearAccelZIndex    = 22; // Inst. G. Signed Thousandths
                constexpr uint8_t kCalMagXIndex         = 24; // Int16 (Device Units)
                constexpr uint8_t kCalMagYIndex         = 26; // Int16 (Device Units)
                constexpr uint8_t kCalMagZIndex         = 28; // Int16 (Device Units)
                constexpr uint8_t kCalMagNormRatioIndex = 30; // Ratio. Unsigned Hundredths
                constexpr uint8_t kCalMagScalarIndex    = 32; // Coefficent. Signed q16:16
                constexpr uint8_t kMpuTempIndex         = 36; // Centigrade. Signed Hundredths
                constexpr uint8_t kRawMagXIndex         = 38; // Int16 (Device Units)
                constexpr uint8_t kRawMagYIndex         = 40; // Int16 (Device Units)
                constexpr uint8_t kRawMagZIndex         = 42; // Int16 (Device Units)
                constexpr uint8_t kQuatWIndex           = 44; // Int16
                constexpr uint8_t kQuatXIndex           = 46; // Int16
                constexpr uint8_t kQuatYIndex           = 48; // Int16
                constexpr uint8_t kQuatZIndex           = 50; // Int16
                constexpr uint8_t kBaroPressureIndex    = 52; // millibar. Signed 16:16
                constexpr uint8_t kBaroTempIndex        = 56; // Centigrade
                constexpr uint8_t kOpstatusIndex        = 58; // NAVX_OP_STATUS_XXX
                constexpr uint8_t kSensorStatusIndex    = 59; // NAVX_SENSOR_STATUS_XXX
                constexpr uint8_t kCalStatusIndex       = 60; // NAVX_CAL_STATUS_XXX
                constexpr uint8_t kSelftestStatusIndex  = 61; // NAVX_SELFTEST_STATUS_XXX
                constexpr uint8_t kChecksumIndex        = 62;
                constexpr uint8_t kTerminatorIndex      = 64;
                constexpr uint8_t kMessageLength        = 66;
            }

            // AHRSAndPositioning Update Packet (similar to AHRS, but removes magnetometer and adds velocity/displacement) */
            namespace AhrsAndPositioningUpdatePacket
            {
                constexpr char kMsgId                  = 'p';
                constexpr uint8_t kYawIndex            = 4;  // Degrees.  Signed Hundredths
                constexpr uint8_t kRollIndex           = 6;  // Degrees.  Signed Hundredths
                constexpr uint8_t kPitchIndex          = 8;  // Degrees.  Signed Hundredths
                constexpr uint8_t kHeadingIndex        = 10; // Degrees.  Unsigned Hundredths
                constexpr uint8_t kAltitudeIndex       = 12; // Meters.   Signed 16:16
                constexpr uint8_t kFusedHeadingIndex   = 16; // Degrees.  Unsigned Hundredths
                constexpr uint8_t kLinearAccelXIndex   = 18; // Inst. G.  Signed Thousandths
                constexpr uint8_t kLinearAccelYIndex   = 20; // Inst. G.  Signed Thousandths
                constexpr uint8_t kLinearAccelZIndex   = 22; // Inst. G.  Signed Thousandths
                constexpr uint8_t kVelocityXIndex      = 24; // Signed 16:16, in meters/sec
                constexpr uint8_t kVelocityYIndex      = 28; // Signed 16:16, in meters/sec
                constexpr uint8_t kVelocityZIndex      = 32; // Signed 16:16, in meters/sec
                constexpr uint8_t kDisplacementXIndex  = 36; // Signed 16:16, in meters
                constexpr uint8_t kDisplacementYIndex  = 40; // Signed 16:16, in meters
                constexpr uint8_t kDisplacementZIndex  = 44; // Signed 16:16, in meters
                constexpr uint8_t kQuatWIndex          = 48; // INT16
                constexpr uint8_t kQuatXIndex          = 50; // INT16
                constexpr uint8_t kQuatYIndex          = 52; // INT16
                constexpr uint8_t kQuatZIndex          = 54; // INT16
                constexpr uint8_t kMpuTempValueIndex   = 56; // Centigrade.  Signed Hundredths
                constexpr uint8_t kOpstatusIndex       = 58; // NAVX_OP_STATUS_XXX
                constexpr uint8_t kSensorStatusIndex   = 59; // NAVX_SENSOR_STATUS_XXX
                constexpr uint8_t kCalStatusIndex      = 60; // NAVX_CAL_STATUS_XXX
                constexpr uint8_t kSelftestStatusIndex = 61; // NAVX_SELFTEST_STATUS_XXX
                constexpr uint8_t kChecksumIndex       = 62;
                constexpr uint8_t kTerminatorIndex     = 64;
                constexpr uint8_t kMessageLength       = 66;
            }

            // AHRSAndPositioningWithTimestamp (similar to AhrsPos, but adds sample timestamp */
            namespace AhrsAndPositioningWithTimestamp
            {
                constexpr char kMsgId                  = 't';
                constexpr uint8_t kYawIndex            = 4;  // Signed 16:16.  Signed Hundredths
                constexpr uint8_t kRollIndex           = 8;  // Signed 16:16.  Signed Hundredths
                constexpr uint8_t kPitchIndex          = 12; // Signed 16:16.  Signed Hundredths
                constexpr uint8_t kHeadingIndex        = 16; // Signed 16:16.  Unsigned Hundredths
                constexpr uint8_t kAltitudeIndex       = 20; // Meters.   Signed 16:16
                constexpr uint8_t kFusedHeadingIndex   = 24; // Degrees.  Unsigned Hundredths
                constexpr uint8_t kLinearAccelXIndex   = 28; // Inst. G.  Signed Thousandths
                constexpr uint8_t kLinearAccelYIndex   = 32; // Inst. G.  Signed Thousandths
                constexpr uint8_t kLinearAccelZIndex   = 36; // Inst. G.  Signed Thousandths
                constexpr uint8_t kVelocityXIndex      = 40; // Signed 16:16, in meters/sec
                constexpr uint8_t kVelocityYIndex      = 44; // Signed 16:16, in meters/sec
                constexpr uint8_t kVelocityZIndex      = 48; // Signed 16:16, in meters/sec
                constexpr uint8_t kDisplacementXIndex  = 52; // Signed 16:16, in meters
                constexpr uint8_t kDisplacementYIndex  = 56; // Signed 16:16, in meters
                constexpr uint8_t kDisplacementZIndex  = 60; // Signed 16:16, in meters
                constexpr uint8_t kQuatWIndex          = 64; // INT16
                constexpr uint8_t kQuatXIndex          = 68; // INT16
                constexpr uint8_t kQuatYIndex          = 72; // INT16
                constexpr uint8_t kQuatZIndex          = 76; // INT16
                constexpr uint8_t kMpuTempValueIndex   = 80; // Centigrade.  Signed Hundredths
                constexpr uint8_t kOpstatusIndex       = 82; // NAVX_OP_STATUS_XXX
                constexpr uint8_t kSensorStatusIndex   = 83; // NAVX_SENSOR_STATUS_XXX
                constexpr uint8_t kCalStatusIndex      = 84; // NAVX_CAL_STATUS_XXX
                constexpr uint8_t kSelftestStatusIndex = 85; // NAVX_SELFTEST_STATUS_XXX
                constexpr uint8_t kTimestampIndex      = 86; // uint32 Timestamp in milliseconds
                constexpr uint8_t kChecksumIndex       = 90;
                constexpr uint8_t kTerminatorIndex     = 92;
                constexpr uint8_t kMessageLength       = 94;
            }

            // Data Get Request: Tuning Variable, Mag Cal, Board Identity (Response message depends on request type)
            namespace DataGetRequest
            {
                constexpr char kMsgId              = 'D';
                constexpr uint8_t kDataTypeIndex   = 4;
                constexpr uint8_t kVariableIdIndex = 5;
                constexpr uint8_t kChecksumIndex   = 6;
                constexpr uint8_t kTerminatorIndex = 8;
                constexpr uint8_t kMessageLength   = 10;
            }

            // Data Set Response Packet (in response to MagCal SET and Tuning SET m)
            namespace DataSetResponse
            {
                constexpr char kMsgId              = 'v';
                constexpr uint8_t kDataTypeIndex   = 4;
                constexpr uint8_t kVariableIdIndex = 5;
                constexpr uint8_t kStatusIndex     = 6;
                constexpr uint8_t kChecksumIndex   = 7;
                constexpr uint8_t kTerminatorIndex = 9;
                constexpr uint8_t kMessageLength   = 11;
            }

            // Integration Control Command Packet
            / namespace IntegrationControlCommand
            {
                constexpr char kMsgId              = 'I';
                constexpr uint8_t kActionIndex     = 4;
                constexpr uint8_t kParameterIndex = 5;
                constexpr uint8_t kChecksumIndex   = 9;
                constexpr uint8_t kTerminatorIndex = 11;
                constexpr uint8_t kMessageLength   = 13;
            }

            // Integration Control Response Packet
            namespace IntegrationControlResponse
            {
                constexpr char kMsgId              = 'j';
                constexpr uint8_t kActionIndex     = 4;
                constexpr uint8_t kParameterINdex  = 5;
                constexpr uint8_t kChecksumIndex   = 9;
                constexpr uint8_t kTerminatorIndex = 11;
                constexpr uint8_t kMessageLength   = 13;
            }

            // Magnetometer Calibration Packet
            // This message may be used to set a new calibration to the NavX board
            // or it may be used to retrieve the current calibration


            class AhrsProtocol : public ImuProtocol
            {
                struct AhrsUpdateBase
                {
                    float yaw;
                    float pitch;
                    float roll;
                    float compass_heading;
                    float altitude;
                    float fused_heading;
                    float linear_accel_x;
                    float linear_accel_y;
                    float linear_accel_z;
                    float quat_w;
                    float quat_x;
                    float quat_y;
                    float quat_z;
                    float baro_pressure;
                    float baro_temp;
                    uint8_t op_status;
                    uint8_t sensor_status;
                    uint8_t cal_status;
                    uint8_t selftest_status;
                };
            public:

                struct AhrsUpdate : public AhrsUpdateBase
                {
                    short cal_mag_x;
                    short cal_mag_y;
                    short cal_mag_z;
                    float mag_field_norm_ratio;
                    float mag_field_norm_scalar;
                    short raw_mag_x;
                    short raw_mag_y;
                    short raw_mag_z;
                };

                struct AhrsPosUpdate : public AhrsUpdateBase
                {
                    float velocity_x;
                    float velocity_y;
                    float velocity_z;
                    float displacement_x;
                    float displacement_y;
                    float displacement_z;

                };

                struct AhrsPosTSUpdate: public AhrsPosUpdate
                {
                    uint32_t timestamp;
                };

                struct BoardId
                {
                    uint8_t type;
                    uint8_t hw_rev;
                    uint8_t firmware_version_major;
                    uint8_t firmware_version_minor;
                    uint8_t firmware_revision;
                    uint8_t unique_id[2];
                };

                struct IntegrationControl
                {
                    uint8_t action;
                    int parameter;
                };

                static std::array<char, IntegrationControlCommand::kMessageLength> encodeIntegrationControlCmd(const IntegrationControl& cmd)
                {
                    std::array<char, IntegrationControlCommand::kMessageLength> buffer;
                    // Header
                    buffer[0] = kPacketStartChar;
                    buffer[1] = kBinaryPacketIndicatorChar;
                    buffer[2] = IntegrationControlCommand::kMessageLength - 2;
                    buffer[3] = IntegrationControlCommand::kMsgId;

                    // Data
                    buffer[IntegrationControlCommand::kActionIndex] = cmd.action;
                    ImuRegisters::encoderProtocolInt32(cmd.parameter, &buffer[IntegrationControlCommand::kParameterIndex]);

                    // Footer
                    encodeTermination(buffer, IntegrationControlCommand::kMessageLength, IntegrationControlCommand::kMessageLength - 4);

                    return buffer;
                }

                static IntegrationControl decodeIntegrationControlCmd(std::array<char, IntegrationControlResponse::kMessageLength> buffer)
                {
                    if ((buffer[0] != kPacketStartChar) ||
                            (buffer[1] != kBinaryPacketIndicatorChar) ||
                            (buffer[2] != IntegrationControlCommand::kMessageLength - 2) ||
                            (buffer[3] != IntegrationControlCommand::kMsgId))
                    {
                        // todo(Andrew): throw exception

                    }
                    if (!verifyChecksum(buffer, IntegrationControlCommand::kChecksumIndex))
                    {
                        // todo(Andrew): throw exception
                    }

                    // Data
                    IntegrationControl cmd;
                    cmd.action = static_cast<uint8_t>(buffer[IntegrationControlCommand::kActionIndex]);
                    cmd.parameter = ImuRegisters::decodeProtocolInt32(&buffer[IntegrationControlCommand::kParameterIndex]);
                    return cmd;
                }

                static std::array<char, IntegrationControlResponse::kMessageLength> encodeIntegrationControlResponse(const IntegrationControl& cmd)
                {
                    std::array<char, IntegrationControlResponse::kMessageLength> buffer;

                    // Header
                    buffer[0] = kPacketStartChar;
                    buffer[1] = kBinaryPacketIndicatorChar;
                    buffer[2] = IntegrationControlResponse::kMessageLength - 2;
                    buffer[3] = IntegrationControlResponse::kMsgId;

                    // Data
                    buffer[IntegrationControlResponse::kActionIndex] = cmd.action;
                    ImuRegister::encoderProtocolInt32(buffer, &buffer[IntegrationControlResponse::kParameterIndex]);

                    // Footer
                    encodeTermination(buffer, IntegrationControlResponse::kMessageLength, IntegrationControlResponse::kMessageLength - 4);

                    return buffer;
                }

                static IntegrationControl decodeIntegrationControlResponse(std::array<char, IntegrationControlResponse::kMessageLength> buffer)
                {
                    if ((buffer[0] != kPacketStartChar) ||
                            (buffer[1] != kBinaryPacketIndicatorChar) ||
                            (buffer[2] != IntegrationControlResponse::kMessageLength - 2) ||
                            (buffer[3] != IntegrationControlResponse::kMsgId))
                    {
                        // todo(Andrew): throw exception

                    }
                    if (!verifyChecksum(buffer, IntegrationControlResponse::kChecksumIndex))
                    {
                        // todo(Andrew): throw exception
                    }

                    // Data
                    IntegrationControl cmd;
                    cmd.action = static_cast<uint8_t>(buffer[IntegrationControlResponse::kActionIndex]);
                    cmd.parameter = ImuRegisters::decodeProtocolInt32(&buffer[IntegrationControlResponse::kParameterIndex]);
                    return cmd;
                }

                static std::array <char,> encodeTuningVariableCmd(DataAction getset, TuningVarId id, float val)
                {
                    std::array<char, > buffer;

                    // Header
                    buffer[0] = kPacketStartChar;
                    buffer[1] = kBinaryPacketIndicatorChar;
                    buffer[2] = FusionTuning::kMessageLength - 2;
                    buffer[3] = FussionTuning::kMsgId;

                    // Data
                    buffer[FusionTuning::kDataActionIndex] = getset;
                    buffer[FusionTuning::kCmdVarIdIndex] = id;
                    ImuRegisters::encodeProtocol1616Float(val, &buffer[FusionTuning::kCmdVarIndex]);

                    // Footer
                    encodeTermination(buffer, FusionTuning::kMessageLength, FusionTuning::kMessageLength - 4);
                    return buffer;
                }

                static decodeTuningVariableCmd()
                {

                }

                static std::array<char, AhrsUpdatePacket::kMessageLength> encodeAhrsUpdate(const AhrsUpdatePacket& packet)
                {
                    std::array<char, AhrsUpdatePacket::kMessageLength> buffer;

                    // Header
                    buffer[0] = kPacketStartChar;
                    buffer[1] = kBinaryPacketIndicatorChar;
                    buffer[2] = AhrsUpdatePacket::kMessageLength - 2;
                    buffer[3] = AhrsUpdatePacket::kMsgId;

                    // Data
                    packet.yaw = ImuRegisters::encodeProtocolSignedHundredthsFloat(&buffer[AhrsUpdatePacket::kYawIndex]);
                    packet.pitch = ImuRegisters::encodeProtocolSignedHundredthsFloat(&buffer[AhrsUpdatePacket::kPitchIndex]);
                    packet.roll = ImuRegisters::encodeProtocolSignedHundredthsFloat(&buffer[AhrsUpdatePacket::kRollIndex]);
                }
            };
        }
    }
}

#endif // AHRS_PROTOCOL_HPP