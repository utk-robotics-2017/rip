#include "ahrs_update.hpp"

namespace rip
{
    namespace navigation
    {
        namespace navx
        {
            AhrsUpdate::AhrsUpdate(const std::string& buffer)
            {

            }

            std::string AhrsUpdate::encode()
            {
                std::string buffer;
                buffer.resize(AhrsUpdatePacket::kMessageLength);

                // Header
                buffer[0] = kPacketStartChar;
                buffer[1] = kBinaryPacketIndicatorChar;
                buffer[2] = AhrsUpdatePacket::kMessageLength - 2;
                buffer[3] = AhrsUpdatePacket::kMsgId;

                // Data
                ImuRegisters::encodeProtocolSignedHundredthsFloat(packet.yaw, &buffer[AhrsUpdatePacket::kYawIndex]);
                ImuRegisters::encodeProtocolSignedHundredthsFloat(packet.pitch, &buffer[AhrsUpdatePacket::kPitchIndex]);
                ImuRegisters::encodeProtocolSignedHundredthsFloat(packet.roll, &buffer[AhrsUpdatePacket::kRollIndex]);
                IMURegisters::encodeProtocolUnsignedHundredthsFloat(packet.compass_heading, &buffer[AhrsUpdatePacket::kHeadingIndex]);
                IMURegisters::encodeProtocol1616Float(packet.altitude, &buffer[AhrsUpdatePacket::kAltitudeIndex]);
                IMURegisters::encodeProtocolUnsignedHundredthsFloat(packet.fused_heading, &buffer[AhrsUpdatePacket::kFusedHeadingIndex]);
                IMURegisters::encodeProtocolSignedThousandthsFloat(packet.linear_accel_x, &buffer[AhrsUpdatePacket::kLinearAccelXIndex]);
                IMURegisters::encodeProtocolSignedThousandthsFloat(packet.linear_accel_y, &buffer[AhrsUpdatePacket::kLinearAccelYIndex]);
                IMURegisters::encodeProtocolSignedThousandthsFloat(packet.linear_accel_z, &buffer[AhrsUpdatePacket::kLinearAccelZIndex]);
                IMURegisters::encodeProtocolInt16(packet.cal_mag_x, &buffer[AhrsUpdatePacket::kCalMagXIndex]);
                IMURegisters::encodeProtocolInt16(packet.cal_mag_y, &buffer[AhrsUpdatePacket::kCalMagYIndex]);
                IMURegisters::encodeProtocolInt16(packet.cal_mag_z, &buffer[AhrsUpdatePacket::kCalMagZIndex]);
                IMURegisters::encodeProtocolUnsignedHundredthsFloat(packet.mag_norm_ratio, &buffer[AhrsUpdatePacket::kCalMagNormRatioIndex]);
                IMURegisters::encodeProtocol1616Float(packet.mag_norm_scalar, &buffer[AhrsUpdatePacket::kCalMagScalarIndex]);
                IMURegisters::encodeProtocolSignedHundredthsFloat(packet.mpu_temp_c, &buffer[AhrsUpdatePacket::kMpuTempIndex]);
                IMURegisters::encodeProtocolInt16(packet.raw_mag_x, &buffer[AhrsUpdatePacket::kRawMagXIndex]);
                IMURegisters::encodeProtocolInt16(packet.raw_mag_y, &buffer[AhrsUpdatePacket::kRawMagYIndex]);
                IMURegisters::encodeProtocolInt16(packet.raw_mag_z, &buffer[AhrsUpdatePacket::kRawMagZIndex]);
                IMURegisters::encodeProtocolInt16(packet.quat_w, &buffer[AhrsUpdatePacket::kQuatWIndex]);
                IMURegisters::encodeProtocolInt16(packet.quat_x, &buffer[AhrsUpdatePacket::kQuatXIndex]);
                IMURegisters::encodeProtocolInt16(packet.quat_y, &buffer[AhrsUpdatePacket::kQuatYIndex]);
                IMURegisters::encodeProtocolInt16(packet.quat_z, &buffer[AhrsUpdatePacket::kQuatZIndex]);
                IMURegisters::encodeProtocol1616Float(packet.baro_pressure, &buffer[AhrsUpdatePacket::kBaroPressureIndex]);
                IMURegisters::encodeProtocolSignedHundredthsFloat(packet.baro_temp_x, &buffer[AhrsUpdatePacket::kBaroTempIndex]);

                buffer[AhrsUpdatePacket::kOpstatusIndex] = packet.op_status;
                buffer[AhrsUpdatePacket::kSensorStatusIndex] = packet.sensor_status;
                buffer[AhrsUpdatePacket::kCalStatusIndex] = packet.cal_status;
                buffer[AhrsUpdatePacket::kSelftestStatusIndex] = packet.selftest_status;

                // Footer
                encodeTermination( buffer, AhrsUpdatePacket::kMessageLength, AhrsUpdatePacket::kMessageLength - 4 );
                return buffer;
            }
        } // navx
    } // navigation
} // rip