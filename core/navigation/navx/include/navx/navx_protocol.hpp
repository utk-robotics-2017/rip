/* ============================================
navX MXP source code is placed under the MIT license
Copyright(c) 2015 Kauai Labs

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files(the "Software"), to deal
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


/*****************************************************************************/
/* This protocol, introduced first with the navX MXP, expands upon the IMU   */
/* protocol by adding the following new functionality:                       */
/*                                                                           */
/* NAVX Update:  Includes Fused Heading and Altitude Info                    */
/* Magnetometer Calibration:  Enables configuration of coefficients from PC  */
/* Board Identity:  Enables retrieval of Board Identification Info           */
/* Fusion Tuning:  Enables configuration of key thresholds/coefficients used */
/*                 in data fusion algorithms from a remote client            */
/*                                                                           */
/* In addition, the navX enable stream command has been extended with a new  */
/* Stream type, in order to enable NAVX Updates.                             */
/*****************************************************************************/
#ifndef _NAVX_PROTOCOL_H_
#define _NAVX_PROTOCOL_H_

#include "imu_protocol.hpp"
#include "imu_registers.hpp"

#include <stdio.h>
#include <stdlib.h>
namespace rip
{
    namespace navigation
    {
        namespace navx
        {
#define BINARY_PACKET_INDICATOR_CHAR '#'

            /* NAVX Protocol encodes certain data in binary format, unlike the IMU  */
            /* protocol, which encodes all data in ASCII characters.  Thus, the     */
            /* packet start and message termination sequences may occur within the  */
            /* message content itself.  To support the binary format, the binary    */
            /* message has this format:                                             */
            /*                                                                      */
            /* [start][binary indicator][len][msgid]<MESSAGE>[checksum][terminator] */
            /*                                                                      */
            /*(The binary indicator and len are not present in the ASCII protocol) */
            /*                                                                      */
            /* The [len] does not include the length of the start and binary        */
            /* indicator characters, but does include all other message items,      */
            /* including the checksum and terminator sequence.                      */

            typedef enum
            {
                UNSPECIFIED = 0,
                MOTION_THRESHOLD = 1,           /* In G */
                YAW_STABLE_THRESHOLD = 2,       /* In Degrees */
                MAG_DISTURBANCE_THRESHOLD = 3,  /* Ratio */
                SEA_LEVEL_PRESSURE = 4,         /* Millibars */
                MIN_TUNING_VAR_ID = MOTION_THRESHOLD,
                MAX_TUNING_VAR_ID = SEA_LEVEL_PRESSURE,
            } NAVX_TUNING_VAR_ID;

            typedef enum
            {
                TUNING_VARIABLE = 0,
                MAG_CALIBRATION = 1,
                BOARD_IDENTITY = 2
            } NAVX_DATA_TYPE;

            typedef enum
            {
                DATA_GET = 0,
                DATA_SET = 1,
                DATA_SET_TO_DEFAULT = 2,
            } NAVX_DATA_ACTION;

#define DATA_GETSET_SUCCESS 0
#define DATA_GETSET_ERROR   1

            // NAVX Update Packet - e.g., !a[yaw][pitch][roll][heading][altitude][fusedheading][accelx/y/z][angular rot x/y/z][opstatus][fusionstatus][cr][lf]

#define MSGID_NAVX_UPDATE 'a'
#define NAVX_UPDATE_YAW_VALUE_INDEX                     4  /* Degrees.  Signed Hundredths */
#define NAVX_UPDATE_ROLL_VALUE_INDEX                    6  /* Degrees.  Signed Hundredths */
#define NAVX_UPDATE_PITCH_VALUE_INDEX                   8  /* Degrees.  Signed Hundredeths */
#define NAVX_UPDATE_HEADING_VALUE_INDEX                 10 /* Degrees.  Unsigned Hundredths */
#define NAVX_UPDATE_ALTITUDE_VALUE_INDEX                12 /* Meters.   Signed 16:16 */
#define NAVX_UPDATE_FUSED_HEADING_VALUE_INDEX           16 /* Degrees.  Unsigned Hundredths */
#define NAVX_UPDATE_LINEAR_ACCEL_X_VALUE_INDEX          18 /* Inst. G.  Signed Thousandths */
#define NAVX_UPDATE_LINEAR_ACCEL_Y_VALUE_INDEX          20 /* Inst. G.  Signed Thousandths */
#define NAVX_UPDATE_LINEAR_ACCEL_Z_VALUE_INDEX          22 /* Inst. G.  Signed Thousandths */
#define NAVX_UPDATE_CAL_MAG_X_VALUE_INDEX               24 /* Int16(Device Units) */
#define NAVX_UPDATE_CAL_MAG_Y_VALUE_INDEX               26 /* Int16(Device Units) */
#define NAVX_UPDATE_CAL_MAG_Z_VALUE_INDEX               28 /* Int16(Device Units) */
#define NAVX_UPDATE_CAL_MAG_NORM_RATIO_VALUE_INDEX      30 /* Ratio.  Unsigned Hundredths */
#define NAVX_UPDATE_CAL_MAG_SCALAR_VALUE_INDEX          32 /* Coefficient. Signed q16:16 */
#define NAVX_UPDATE_MPU_TEMP_VAUE_INDEX                 36 /* Centigrade.  Signed Hundredths */
#define NAVX_UPDATE_RAW_MAG_X_VALUE_INDEX               38 /* INT16(Device Units) */
#define NAVX_UPDATE_RAW_MAG_Y_VALUE_INDEX               40 /* INT16(Device Units) */
#define NAVX_UPDATE_RAW_MAG_Z_VALUE_INDEX               42 /* INT16(Device Units) */
#define NAVX_UPDATE_QUAT_W_VALUE_INDEX                  44 /* INT16 */
#define NAVX_UPDATE_QUAT_X_VALUE_INDEX                  46 /* INT16 */
#define NAVX_UPDATE_QUAT_Y_VALUE_INDEX                  48 /* INT16 */
#define NAVX_UPDATE_QUAT_Z_VALUE_INDEX                  50 /* INT16 */
#define NAVX_UPDATE_BARO_PRESSURE_VALUE_INDEX           52 /* millibar.  Signed 16:16 */
#define NAVX_UPDATE_BARO_TEMP_VAUE_INDEX                56 /* Centigrade.  Signed  Hundredths */
#define NAVX_UPDATE_OPSTATUS_VALUE_INDEX                58 /* NAVX_OP_STATUS_XXX */
#define NAVX_UPDATE_SENSOR_STATUS_VALUE_INDEX           59 /* NAVX_SENSOR_STATUS_XXX */
#define NAVX_UPDATE_CAL_STATUS_VALUE_INDEX              60 /* NAVX_CAL_STATUS_XXX */
#define NAVX_UPDATE_SELFTEST_STATUS_VALUE_INDEX         61 /* NAVX_SELFTEST_STATUS_XXX */
#define NAVX_UPDATE_MESSAGE_CHECKSUM_INDEX              62
#define NAVX_UPDATE_MESSAGE_TERMINATOR_INDEX            64
#define NAVX_UPDATE_MESSAGE_LENGTH                      66

            // NAVXAndPositioning Update Packet(similar to NAVX, but removes magnetometer and adds velocity/displacement) */

#define MSGID_NAVXPOS_UPDATE 'p'
#define NAVXPOS_UPDATE_YAW_VALUE_INDEX                   4 /* Degrees.  Signed Hundredths */
#define NAVXPOS_UPDATE_ROLL_VALUE_INDEX                  6 /* Degrees.  Signed Hundredths */
#define NAVXPOS_UPDATE_PITCH_VALUE_INDEX                 8 /* Degrees.  Signed Hundredeths */
#define NAVXPOS_UPDATE_HEADING_VALUE_INDEX              10 /* Degrees.  Unsigned Hundredths */
#define NAVXPOS_UPDATE_ALTITUDE_VALUE_INDEX             12 /* Meters.   Signed 16:16 */
#define NAVXPOS_UPDATE_FUSED_HEADING_VALUE_INDEX        16 /* Degrees.  Unsigned Hundredths */
#define NAVXPOS_UPDATE_LINEAR_ACCEL_X_VALUE_INDEX       18 /* Inst. G.  Signed Thousandths */
#define NAVXPOS_UPDATE_LINEAR_ACCEL_Y_VALUE_INDEX       20 /* Inst. G.  Signed Thousandths */
#define NAVXPOS_UPDATE_LINEAR_ACCEL_Z_VALUE_INDEX       22 /* Inst. G.  Signed Thousandths */
#define NAVXPOS_UPDATE_VEL_X_VALUE_INDEX                24 /* Signed 16:16, in meters/sec */
#define NAVXPOS_UPDATE_VEL_Y_VALUE_INDEX                28 /* Signed 16:16, in meters/sec */
#define NAVXPOS_UPDATE_VEL_Z_VALUE_INDEX                32 /* Signed 16:16, in meters/sec */
#define NAVXPOS_UPDATE_DISP_X_VALUE_INDEX               36 /* Signed 16:16, in meters */
#define NAVXPOS_UPDATE_DISP_Y_VALUE_INDEX               40 /* Signed 16:16, in meters */
#define NAVXPOS_UPDATE_DISP_Z_VALUE_INDEX               44 /* Signed 16:16, in meters */
#define NAVXPOS_UPDATE_QUAT_W_VALUE_INDEX               48 /* INT16 */
#define NAVXPOS_UPDATE_QUAT_X_VALUE_INDEX               50 /* INT16 */
#define NAVXPOS_UPDATE_QUAT_Y_VALUE_INDEX               52 /* INT16 */
#define NAVXPOS_UPDATE_QUAT_Z_VALUE_INDEX               54 /* INT16 */
#define NAVXPOS_UPDATE_MPU_TEMP_VAUE_INDEX              56 /* Centigrade.  Signed Hundredths */
#define NAVXPOS_UPDATE_OPSTATUS_VALUE_INDEX             58 /* NAVX_OP_STATUS_XXX */
#define NAVXPOS_UPDATE_SENSOR_STATUS_VALUE_INDEX        59 /* NAVX_SENSOR_STATUS_XXX */
#define NAVXPOS_UPDATE_CAL_STATUS_VALUE_INDEX           60 /* NAVX_CAL_STATUS_XXX */
#define NAVXPOS_UPDATE_SELFTEST_STATUS_VALUE_INDEX      61 /* NAVX_SELFTEST_STATUS_XXX */
#define NAVXPOS_UPDATE_MESSAGE_CHECKSUM_INDEX           62
#define NAVXPOS_UPDATE_MESSAGE_TERMINATOR_INDEX         64
#define NAVXPOS_UPDATE_MESSAGE_LENGTH                   66

            // NAVXAndPositioningWithTimestamp Update Packet(similar to NAVXPos, but adds sample timestamp)

#define MSGID_NAVXPOS_TS_UPDATE 't'
#define NAVXPOS_TS_UPDATE_YAW_VALUE_INDEX                4 /* Signed 16:16.  Signed Hundredths */
#define NAVXPOS_TS_UPDATE_ROLL_VALUE_INDEX               8 /* Signed 16:16.  Signed Hundredths */
#define NAVXPOS_TS_UPDATE_PITCH_VALUE_INDEX             12 /* Signed 16:16.  Signed Hundredeths */
#define NAVXPOS_TS_UPDATE_HEADING_VALUE_INDEX           16 /* Signed 16:16.  Unsigned Hundredths */
#define NAVXPOS_TS_UPDATE_ALTITUDE_VALUE_INDEX          20 /* Meters.   Signed 16:16 */
#define NAVXPOS_TS_UPDATE_FUSED_HEADING_VALUE_INDEX     24 /* Degrees.  Unsigned Hundredths */
#define NAVXPOS_TS_UPDATE_LINEAR_ACCEL_X_VALUE_INDEX    28 /* Inst. G.  Signed 16:16 */
#define NAVXPOS_TS_UPDATE_LINEAR_ACCEL_Y_VALUE_INDEX    32 /* Inst. G.  Signed 16:16 */
#define NAVXPOS_TS_UPDATE_LINEAR_ACCEL_Z_VALUE_INDEX    36 /* Inst. G.  Signed 16:16 */
#define NAVXPOS_TS_UPDATE_VEL_X_VALUE_INDEX             40 /* Signed 16:16, in meters/sec */
#define NAVXPOS_TS_UPDATE_VEL_Y_VALUE_INDEX             44 /* Signed 16:16, in meters/sec */
#define NAVXPOS_TS_UPDATE_VEL_Z_VALUE_INDEX             48 /* Signed 16:16, in meters/sec */
#define NAVXPOS_TS_UPDATE_DISP_X_VALUE_INDEX            52 /* Signed 16:16, in meters */
#define NAVXPOS_TS_UPDATE_DISP_Y_VALUE_INDEX            56 /* Signed 16:16, in meters */
#define NAVXPOS_TS_UPDATE_DISP_Z_VALUE_INDEX            60 /* Signed 16:16, in meters */
#define NAVXPOS_TS_UPDATE_QUAT_W_VALUE_INDEX            64 /* Signed 16:16 */
#define NAVXPOS_TS_UPDATE_QUAT_X_VALUE_INDEX            68 /* Signed 16:16 */
#define NAVXPOS_TS_UPDATE_QUAT_Y_VALUE_INDEX            72 /* Signed 16:16 */
#define NAVXPOS_TS_UPDATE_QUAT_Z_VALUE_INDEX            76 /* Signed 16:16 */
#define NAVXPOS_TS_UPDATE_MPU_TEMP_VAUE_INDEX           80 /* Centigrade.  Signed Hundredths */
#define NAVXPOS_TS_UPDATE_OPSTATUS_VALUE_INDEX          82 /* NAVX_OP_STATUS_XXX */
#define NAVXPOS_TS_UPDATE_SENSOR_STATUS_VALUE_INDEX     83 /* NAVX_SENSOR_STATUS_XXX */
#define NAVXPOS_TS_UPDATE_CAL_STATUS_VALUE_INDEX        84 /* NAVX_CAL_STATUS_XXX */
#define NAVXPOS_TS_UPDATE_SELFTEST_STATUS_VALUE_INDEX   85 /* NAVX_SELFTEST_STATUS_XXX */
#define NAVXPOS_TS_UPDATE_TIMESTAMP_INDEX               86 /* UINT32 Timestamp, in milliseconds */
#define NAVXPOS_TS_UPDATE_MESSAGE_CHECKSUM_INDEX        90
#define NAVXPOS_TS_UPDATE_MESSAGE_TERMINATOR_INDEX      92
#define NAVXPOS_TS_UPDATE_MESSAGE_LENGTH                94

            // Data Get Request:  Tuning Variable, Mag Cal, Board Identity(Response message depends upon request type)
#define MSGID_DATA_REQUEST 'D'
#define DATA_REQUEST_DATATYPE_VALUE_INDEX               4
#define DATA_REQUEST_VARIABLEID_VALUE_INDEX             5
#define DATA_REQUEST_CHECKSUM_INDEX                     6
#define DATA_REQUEST_TERMINATOR_INDEX                   8
#define DATA_REQUEST_MESSAGE_LENGTH                     10

            /* Data Set Response Packet(in response to MagCal SET and Tuning SET commands. */
#define MSGID_DATA_SET_RESPONSE 'v'
#define DATA_SET_RESPONSE_DATATYPE_VALUE_INDEX          4
#define DATA_SET_RESPONSE_VARID_VALUE_INDEX             5
#define DATA_SET_RESPONSE_STATUS_VALUE_INDEX            6
#define DATA_SET_RESPONSE_MESSAGE_CHECKSUM_INDEX        7
#define DATA_SET_RESPONSE_MESSAGE_TERMINATOR_INDEX      9
#define DATA_SET_RESPONSE_MESSAGE_LENGTH                11

            /* Integration Control Command Packet */
#define MSGID_INTEGRATION_CONTROL_CMD 'I'
#define INTEGRATION_CONTROL_CMD_ACTION_INDEX                4
#define INTEGRATION_CONTROL_CMD_PARAMETER_INDEX             5
#define INTEGRATION_CONTROL_CMD_MESSAGE_CHECKSUM_INDEX      9
#define INTEGRATION_CONTROL_CMD_MESSAGE_TERMINATOR_INDEX    11
#define INTEGRATION_CONTROL_CMD_MESSAGE_LENGTH              13

            /* Integration Control Response Packet */
#define MSGID_INTEGRATION_CONTROL_RESP 'j'
#define INTEGRATION_CONTROL_RESP_ACTION_INDEX               4
#define INTEGRATION_CONTROL_RESP_PARAMETER_INDEX            5
#define INTEGRATION_CONTROL_RESP_MESSAGE_CHECKSUM_INDEX     9
#define INTEGRATION_CONTROL_RESP_MESSAGE_TERMINATOR_INDEX   11
#define INTEGRATION_CONTROL_RESP_MESSAGE_LENGTH             13

            /* Magnetometer Calibration Packet
             ** This message may be used to SET(store) a new calibration into the navX board, or may be used
             ** to retrieve the current calibration data from the navX board. */
#define MSGID_MAG_CAL_CMD 'M'
#define MAG_CAL_DATA_ACTION_VALUE_INDEX                 4
#define MAG_X_BIAS_VALUE_INDEX                          5 /* signed short */
#define MAG_Y_BIAS_VALUE_INDEX                          7
#define MAG_Z_BIAS_VALUE_INDEX                          9
#define MAG_XFORM_1_1_VALUE_INDEX                       11 /* signed 16:16 */
#define MAG_XFORM_1_2_VALUE_INDEX                       15
#define MAG_XFORM_1_3_VALUE_INDEX                       19
#define MAG_XFORM_2_1_VALUE_INDEX                       23
#define MAG_XFORM_2_2_VALUE_INDEX                       27
#define MAG_XFORM_2_3_VALUE_INDEX                       31
#define MAG_XFORM_3_1_VALUE_INDEX                       35
#define MAG_XFORM_3_2_VALUE_INDEX                       39
#define MAG_XFORM_3_3_VALUE_INDEX                       43
#define MAG_CAL_EARTH_MAG_FIELD_NORM_VALUE_INDEX        47
#define MAG_CAL_CMD_MESSAGE_CHECKSUM_INDEX              51
#define MAG_CAL_CMD_MESSAGE_TERMINATOR_INDEX            53
#define MAG_CAL_CMD_MESSAGE_LENGTH                      55

            /* Tuning Variable Packet
             ** This message may be used to SET(modify) a tuning variable into the navX board,
             ** or to retrieve a current tuning variable from the navX board. */
#define MSGID_FUSION_TUNING_CMD 'T'
#define FUSION_TUNING_DATA_ACTION_VALUE_INDEX           4
#define FUSION_TUNING_CMD_VAR_ID_VALUE_INDEX            5
#define FUSION_TUNING_CMD_VAR_VALUE_INDEX               6
#define FUSION_TUNING_CMD_MESSAGE_CHECKSUM_INDEX        10
#define FUSION_TUNING_CMD_MESSAGE_TERMINATOR_INDEX      12
#define FUSION_TUNING_CMD_MESSAGE_LENGTH                14

            // Board Identity Response Packet
            // Sent in response to a Data Get(Board ID) message
#define MSGID_BOARD_IDENTITY_RESPONSE 'i'
#define BOARD_IDENTITY_BOARDTYPE_VALUE_INDEX            4
#define BOARD_IDENTITY_HWREV_VALUE_INDEX                5
#define BOARD_IDENTITY_FW_VER_MAJOR                     6
#define BOARD_IDENTITY_FW_VER_MINOR                     7
#define BOARD_IDENTITY_FW_VER_REVISION_VALUE_INDEX      8
#define BOARD_IDENTITY_UNIQUE_ID_0                      10
#define BOARD_IDENTITY_UNIQUE_ID_1                      11
#define BOARD_IDENTITY_UNIQUE_ID_2                      12
#define BOARD_IDENTITY_UNIQUE_ID_3                      13
#define BOARD_IDENTITY_UNIQUE_ID_4                      14
#define BOARD_IDENTITY_UNIQUE_ID_5                      15
#define BOARD_IDENTITY_UNIQUE_ID_6                      16
#define BOARD_IDENTITY_UNIQUE_ID_7                      17
#define BOARD_IDENTITY_UNIQUE_ID_8                      18
#define BOARD_IDENTITY_UNIQUE_ID_9                      19
#define BOARD_IDENTITY_UNIQUE_ID_10                     20
#define BOARD_IDENTITY_UNIQUE_ID_11                     21
#define BOARD_IDENTITY_RESPONSE_CHECKSUM_INDEX          22
#define BOARD_IDENTITY_RESPONSE_TERMINATOR_INDEX        24
#define BOARD_IDENTITY_RESPONSE_MESSAGE_LENGTH          26

#define NAVX_PROTOCOL_MAX_MESSAGE_LENGTH NAVX_UPDATE_MESSAGE_LENGTH

            class NavXProtocol : public IMUProtocol
            {
            public:

                struct NavXUpdateBase
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
                    float mpu_temp;
                    float quat_w;
                    float quat_x;
                    float quat_y;
                    float quat_z;
                    float barometric_pressure;
                    float baro_temp;
                    uint8_t op_status;
                    uint8_t  sensor_status;
                    uint8_t  cal_status;
                    uint8_t  selftest_status;
                };

                struct NavXUpdate : public NavXUpdateBase
                {
                    short   cal_mag_x;
                    short   cal_mag_y;
                    short   cal_mag_z;
                    float   mag_field_norm_ratio;
                    float   mag_field_norm_scalar;
                    short   raw_mag_x;
                    short   raw_mag_y;
                    short   raw_mag_z;
                };

                struct NavXPosUpdate : public NavXUpdateBase
                {
                    float   vel_x;
                    float   vel_y;
                    float   vel_z;
                    float   disp_x;
                    float   disp_y;
                    float   disp_z;
                };

                struct NavXPosTSUpdate : public NavXPosUpdate
                {
                    uint32_t timestamp;
                };

                struct BoardID
                {
                    uint8_t type;
                    uint8_t hw_rev;
                    uint8_t fw_ver_major;
                    uint8_t fw_ver_minor;
                    int16_t fw_revision;
                    uint8_t unique_id[12];
                };

                struct IntegrationControl
                {
                    uint8_t action;
                    int     parameter;
                };

            public:

                static int encodeIntegrationControlCmd(char* protocol_buffer, struct IntegrationControl& cmd)
                {
                    // Header
                    protocol_buffer[0] = PACKET_START_CHAR;
                    protocol_buffer[1] = BINARY_PACKET_INDICATOR_CHAR;
                    protocol_buffer[2] = INTEGRATION_CONTROL_CMD_MESSAGE_LENGTH - 2;
                    protocol_buffer[3] = MSGID_INTEGRATION_CONTROL_CMD;
                    // Data
                    protocol_buffer[INTEGRATION_CONTROL_CMD_ACTION_INDEX] = cmd.action;
                    IMURegisters::encodeProtocolInt32(cmd.parameter, &protocol_buffer[INTEGRATION_CONTROL_CMD_PARAMETER_INDEX]);
                    // Footer
                    encodeTermination(protocol_buffer, INTEGRATION_CONTROL_CMD_MESSAGE_LENGTH, INTEGRATION_CONTROL_CMD_MESSAGE_LENGTH - 4);
                    return INTEGRATION_CONTROL_CMD_MESSAGE_LENGTH;
                }

                static int decodeIntegrationControlCmd(char* buffer, int length, uint8_t& action, int32_t& parameter)
                {
                    if (length < INTEGRATION_CONTROL_CMD_MESSAGE_LENGTH) { return 0; }
                    if ((buffer[0] == PACKET_START_CHAR) &&
                            (buffer[1] == BINARY_PACKET_INDICATOR_CHAR) &&
                            (buffer[2] == INTEGRATION_CONTROL_CMD_MESSAGE_LENGTH - 2) &&
                            (buffer[3] == MSGID_INTEGRATION_CONTROL_CMD))
                    {
                        if (!verifyChecksum(buffer, INTEGRATION_CONTROL_CMD_MESSAGE_CHECKSUM_INDEX)) { return 0; }

                        // Data
                        action = (uint8_t)buffer[INTEGRATION_CONTROL_CMD_ACTION_INDEX];
                        parameter = IMURegisters::decodeProtocolInt32(&buffer[INTEGRATION_CONTROL_CMD_PARAMETER_INDEX]);
                        return INTEGRATION_CONTROL_CMD_MESSAGE_LENGTH;
                    }
                    return 0;
                }

                static int encodeIntegrationControlResponse(char* protocol_buffer, uint8_t action, int32_t parameter)
                {
                    // Header
                    protocol_buffer[0] = PACKET_START_CHAR;
                    protocol_buffer[1] = BINARY_PACKET_INDICATOR_CHAR;
                    protocol_buffer[2] = INTEGRATION_CONTROL_RESP_MESSAGE_LENGTH - 2;
                    protocol_buffer[3] = MSGID_INTEGRATION_CONTROL_RESP;
                    // Data
                    protocol_buffer[INTEGRATION_CONTROL_RESP_ACTION_INDEX] = action;
                    IMURegisters::encodeProtocolInt32(parameter, &protocol_buffer[INTEGRATION_CONTROL_RESP_PARAMETER_INDEX]);
                    // Footer
                    encodeTermination(protocol_buffer, INTEGRATION_CONTROL_RESP_MESSAGE_LENGTH, INTEGRATION_CONTROL_RESP_MESSAGE_LENGTH - 4);
                    return INTEGRATION_CONTROL_RESP_MESSAGE_LENGTH;
                }

                static int decodeIntegrationControlResponse(char* buffer, int length, struct IntegrationControl& rsp)
                {
                    if (length < INTEGRATION_CONTROL_RESP_MESSAGE_LENGTH) { return 0; }
                    if ((buffer[0] == PACKET_START_CHAR) &&
                            (buffer[1] == BINARY_PACKET_INDICATOR_CHAR) &&
                            (buffer[2] == INTEGRATION_CONTROL_RESP_MESSAGE_LENGTH - 2) &&
                            (buffer[3] == MSGID_INTEGRATION_CONTROL_RESP))
                    {
                        if (!verifyChecksum(buffer, INTEGRATION_CONTROL_RESP_MESSAGE_CHECKSUM_INDEX)) { return 0; }
                        //RIP TODO: ADD exception handling BEHN
                        // Data
                        rsp.action = (uint8_t)buffer[INTEGRATION_CONTROL_RESP_ACTION_INDEX];
                        rsp.parameter = IMURegisters::decodeProtocolInt32(&buffer[INTEGRATION_CONTROL_RESP_PARAMETER_INDEX]);
                        return INTEGRATION_CONTROL_RESP_MESSAGE_LENGTH;
                    }
                    return 0;
                }

                static int encodeTuningVariableCmd(char* protocol_buffer, NAVX_DATA_ACTION getset, NAVX_TUNING_VAR_ID id, float val)
                {
                    // Header
                    protocol_buffer[0] = PACKET_START_CHAR;
                    protocol_buffer[1] = BINARY_PACKET_INDICATOR_CHAR;
                    protocol_buffer[2] = FUSION_TUNING_CMD_MESSAGE_LENGTH - 2;
                    protocol_buffer[3] = MSGID_FUSION_TUNING_CMD;
                    // Data
                    protocol_buffer[FUSION_TUNING_DATA_ACTION_VALUE_INDEX] = getset;
                    protocol_buffer[FUSION_TUNING_CMD_VAR_ID_VALUE_INDEX] = id;
                    IMURegisters::encodeProtocol1616Float(val, &protocol_buffer[FUSION_TUNING_CMD_VAR_VALUE_INDEX]);
                    // Footer
                    encodeTermination(protocol_buffer, FUSION_TUNING_CMD_MESSAGE_LENGTH, FUSION_TUNING_CMD_MESSAGE_LENGTH - 4);
                    return FUSION_TUNING_CMD_MESSAGE_LENGTH;
                }

                static int decodeTuningVariableCmd(char* buffer, int length, NAVX_DATA_ACTION& getset, NAVX_TUNING_VAR_ID& id, float& val)
                {
                    if (length < FUSION_TUNING_CMD_MESSAGE_LENGTH) { return 0; }
                    if ((buffer[0] == PACKET_START_CHAR) &&
                            (buffer[1] == BINARY_PACKET_INDICATOR_CHAR) &&
                            (buffer[2] == FUSION_TUNING_CMD_MESSAGE_LENGTH - 2) &&
                            (buffer[3] == MSGID_FUSION_TUNING_CMD))
                    {
                        if (!verifyChecksum(buffer, FUSION_TUNING_CMD_MESSAGE_CHECKSUM_INDEX)) { return 0; }

                        // Data
                        getset = (NAVX_DATA_ACTION)buffer[FUSION_TUNING_DATA_ACTION_VALUE_INDEX];
                        id = (NAVX_TUNING_VAR_ID)buffer[FUSION_TUNING_CMD_VAR_ID_VALUE_INDEX];
                        val = IMURegisters::decodeProtocol1616Float(&buffer[FUSION_TUNING_CMD_VAR_VALUE_INDEX]);
                        return FUSION_TUNING_CMD_MESSAGE_LENGTH;
                    }
                    return 0;
                }

                static int encodeNavXUpdate(char* protocol_buffer,
                                            float yaw, float pitch, float roll,
                                            float compass_heading, float altitude, float fused_heading,
                                            float linear_accel_x, float linear_accel_y, float linear_accel_z,
                                            float mpu_temp_c,
                                            int16_t raw_mag_x, int16_t raw_mag_y, int16_t raw_mag_z,
                                            int16_t cal_mag_x, int16_t cal_mag_y, int16_t cal_mag_z,
                                            float mag_norm_ratio, float mag_norm_scalar,
                                            int16_t quat_w, int16_t quat_x, int16_t quat_y, int16_t quat_z,
                                            float baro_pressure, float baro_temp_c,
                                            uint8_t op_status, uint8_t sensor_status,
                                            uint8_t cal_status, uint8_t selftest_status)
                {
                    // Header
                    protocol_buffer[0] = PACKET_START_CHAR;
                    protocol_buffer[1] = BINARY_PACKET_INDICATOR_CHAR;
                    protocol_buffer[2] = NAVX_UPDATE_MESSAGE_LENGTH - 2;
                    protocol_buffer[3] = MSGID_NAVX_UPDATE;
                    // data
                    IMURegisters::encodeProtocolSignedHundredthsFloat(yaw, &protocol_buffer[NAVX_UPDATE_YAW_VALUE_INDEX]);
                    IMURegisters::encodeProtocolSignedHundredthsFloat(pitch, &protocol_buffer[NAVX_UPDATE_PITCH_VALUE_INDEX]);
                    IMURegisters::encodeProtocolSignedHundredthsFloat(roll, &protocol_buffer[NAVX_UPDATE_ROLL_VALUE_INDEX]);
                    IMURegisters::encodeProtocolUnsignedHundredthsFloat(compass_heading, &protocol_buffer[NAVX_UPDATE_HEADING_VALUE_INDEX]);
                    IMURegisters::encodeProtocol1616Float(altitude, &protocol_buffer[NAVX_UPDATE_ALTITUDE_VALUE_INDEX]);
                    IMURegisters::encodeProtocolUnsignedHundredthsFloat(fused_heading, &protocol_buffer[NAVX_UPDATE_FUSED_HEADING_VALUE_INDEX]);
                    IMURegisters::encodeProtocolSignedThousandthsFloat(linear_accel_x, &protocol_buffer[NAVX_UPDATE_LINEAR_ACCEL_X_VALUE_INDEX]);
                    IMURegisters::encodeProtocolSignedThousandthsFloat(linear_accel_y, &protocol_buffer[NAVX_UPDATE_LINEAR_ACCEL_Y_VALUE_INDEX]);
                    IMURegisters::encodeProtocolSignedThousandthsFloat(linear_accel_z, &protocol_buffer[NAVX_UPDATE_LINEAR_ACCEL_Z_VALUE_INDEX]);
                    IMURegisters::encodeProtocolInt16(cal_mag_x, &protocol_buffer[NAVX_UPDATE_CAL_MAG_X_VALUE_INDEX]);
                    IMURegisters::encodeProtocolInt16(cal_mag_y, &protocol_buffer[NAVX_UPDATE_CAL_MAG_Y_VALUE_INDEX]);
                    IMURegisters::encodeProtocolInt16(cal_mag_z, &protocol_buffer[NAVX_UPDATE_CAL_MAG_Z_VALUE_INDEX]);
                    IMURegisters::encodeProtocolUnsignedHundredthsFloat(mag_norm_ratio, &protocol_buffer[NAVX_UPDATE_CAL_MAG_NORM_RATIO_VALUE_INDEX]);
                    IMURegisters::encodeProtocol1616Float(mag_norm_scalar, &protocol_buffer[NAVX_UPDATE_CAL_MAG_SCALAR_VALUE_INDEX]);
                    IMURegisters::encodeProtocolSignedHundredthsFloat(mpu_temp_c, &protocol_buffer[NAVX_UPDATE_MPU_TEMP_VAUE_INDEX]);
                    IMURegisters::encodeProtocolInt16(raw_mag_x, &protocol_buffer[NAVX_UPDATE_RAW_MAG_X_VALUE_INDEX]);
                    IMURegisters::encodeProtocolInt16(raw_mag_y, &protocol_buffer[NAVX_UPDATE_RAW_MAG_Y_VALUE_INDEX]);
                    IMURegisters::encodeProtocolInt16(raw_mag_z, &protocol_buffer[NAVX_UPDATE_RAW_MAG_Z_VALUE_INDEX]);
                    IMURegisters::encodeProtocolInt16(quat_w, &protocol_buffer[NAVX_UPDATE_QUAT_W_VALUE_INDEX]);
                    IMURegisters::encodeProtocolInt16(quat_x, &protocol_buffer[NAVX_UPDATE_QUAT_X_VALUE_INDEX]);
                    IMURegisters::encodeProtocolInt16(quat_y, &protocol_buffer[NAVX_UPDATE_QUAT_Y_VALUE_INDEX]);
                    IMURegisters::encodeProtocolInt16(quat_z, &protocol_buffer[NAVX_UPDATE_QUAT_Z_VALUE_INDEX]);
                    IMURegisters::encodeProtocol1616Float(baro_pressure, &protocol_buffer[NAVX_UPDATE_BARO_PRESSURE_VALUE_INDEX]);
                    IMURegisters::encodeProtocolSignedHundredthsFloat(baro_temp_c, &protocol_buffer[NAVX_UPDATE_BARO_TEMP_VAUE_INDEX]);

                    protocol_buffer[NAVX_UPDATE_OPSTATUS_VALUE_INDEX] = op_status;
                    protocol_buffer[NAVX_UPDATE_SENSOR_STATUS_VALUE_INDEX] = sensor_status;
                    protocol_buffer[NAVX_UPDATE_CAL_STATUS_VALUE_INDEX] = cal_status;
                    protocol_buffer[NAVX_UPDATE_SELFTEST_STATUS_VALUE_INDEX] = selftest_status;
                    // Footer
                    encodeTermination(protocol_buffer, NAVX_UPDATE_MESSAGE_LENGTH, NAVX_UPDATE_MESSAGE_LENGTH - 4);
                    return NAVX_UPDATE_MESSAGE_LENGTH;
                }

                static int decodeNavXUpdate(char* buffer, int length, struct NavXUpdate& update)
                {
                    if (length < NAVX_UPDATE_MESSAGE_LENGTH) { return 0; }
                    if ((buffer[0] == PACKET_START_CHAR) &&
                            (buffer[1] == BINARY_PACKET_INDICATOR_CHAR) &&
                            (buffer[2] == NAVX_UPDATE_MESSAGE_LENGTH - 2) &&
                            (buffer[3] == MSGID_NAVX_UPDATE))
                    {

                        if (!verifyChecksum(buffer, NAVX_UPDATE_MESSAGE_CHECKSUM_INDEX)) { return 0; }

                        update.yaw = IMURegisters::decodeProtocolSignedHundredthsFloat(&buffer[NAVX_UPDATE_YAW_VALUE_INDEX]);
                        update.pitch = IMURegisters::decodeProtocolSignedHundredthsFloat(&buffer[NAVX_UPDATE_PITCH_VALUE_INDEX]);
                        update.roll = IMURegisters::decodeProtocolSignedHundredthsFloat(&buffer[NAVX_UPDATE_ROLL_VALUE_INDEX]);
                        update.compass_heading = IMURegisters::decodeProtocolUnsignedHundredthsFloat(&buffer[NAVX_UPDATE_HEADING_VALUE_INDEX]);
                        update.altitude = IMURegisters::decodeProtocol1616Float(&buffer[NAVX_UPDATE_ALTITUDE_VALUE_INDEX]);
                        update.fused_heading = IMURegisters::decodeProtocolUnsignedHundredthsFloat(&buffer[NAVX_UPDATE_FUSED_HEADING_VALUE_INDEX]);
                        update.linear_accel_x = IMURegisters::decodeProtocolSignedThousandthsFloat(&buffer[NAVX_UPDATE_LINEAR_ACCEL_X_VALUE_INDEX]);
                        update.linear_accel_y = IMURegisters::decodeProtocolSignedThousandthsFloat(&buffer[NAVX_UPDATE_LINEAR_ACCEL_Y_VALUE_INDEX]);
                        update.linear_accel_z = IMURegisters::decodeProtocolSignedThousandthsFloat(&buffer[NAVX_UPDATE_LINEAR_ACCEL_Z_VALUE_INDEX]);
                        update.cal_mag_x = IMURegisters::decodeProtocolInt16(&buffer[NAVX_UPDATE_CAL_MAG_X_VALUE_INDEX]);
                        update.cal_mag_y = IMURegisters::decodeProtocolInt16(&buffer[NAVX_UPDATE_CAL_MAG_Y_VALUE_INDEX]);
                        update.cal_mag_z = IMURegisters::decodeProtocolInt16(&buffer[NAVX_UPDATE_CAL_MAG_Z_VALUE_INDEX]);
                        update.mag_field_norm_ratio = IMURegisters::decodeProtocolUnsignedHundredthsFloat(&buffer[NAVX_UPDATE_CAL_MAG_NORM_RATIO_VALUE_INDEX]);
                        update.mag_field_norm_scalar = IMURegisters::decodeProtocol1616Float(&buffer[NAVX_UPDATE_CAL_MAG_SCALAR_VALUE_INDEX]);
                        update.mpu_temp = IMURegisters::decodeProtocolSignedHundredthsFloat(&buffer[NAVX_UPDATE_MPU_TEMP_VAUE_INDEX]);
                        update.raw_mag_x = IMURegisters::decodeProtocolInt16(&buffer[NAVX_UPDATE_RAW_MAG_X_VALUE_INDEX]);
                        update.raw_mag_y = IMURegisters::decodeProtocolInt16(&buffer[NAVX_UPDATE_RAW_MAG_Y_VALUE_INDEX]);
                        update.raw_mag_z = IMURegisters::decodeProtocolInt16(&buffer[NAVX_UPDATE_RAW_MAG_Z_VALUE_INDEX]);
                        /* NAVXPosUpdate:  Quaternions are signed int(16-bit resolution); divide by 16384 to yield +/- 2 radians */
                        update.quat_w = ((float)IMURegisters::decodeProtocolInt16(&buffer[NAVX_UPDATE_QUAT_W_VALUE_INDEX]) / 16384.0f);
                        update.quat_x = ((float)IMURegisters::decodeProtocolInt16(&buffer[NAVX_UPDATE_QUAT_X_VALUE_INDEX]) / 16384.0f);
                        update.quat_y = ((float)IMURegisters::decodeProtocolInt16(&buffer[NAVX_UPDATE_QUAT_Y_VALUE_INDEX]) / 16384.0f);
                        update.quat_z = ((float)IMURegisters::decodeProtocolInt16(&buffer[NAVX_UPDATE_QUAT_Z_VALUE_INDEX]) / 16384.0f);
                        update.barometric_pressure = IMURegisters::decodeProtocol1616Float(&buffer[NAVX_UPDATE_BARO_PRESSURE_VALUE_INDEX]);
                        update.baro_temp = IMURegisters::decodeProtocolSignedHundredthsFloat(&buffer[NAVX_UPDATE_BARO_TEMP_VAUE_INDEX]);
                        update.op_status = buffer[NAVX_UPDATE_OPSTATUS_VALUE_INDEX];
                        update.sensor_status = buffer[NAVX_UPDATE_SENSOR_STATUS_VALUE_INDEX];
                        update.cal_status = buffer[NAVX_UPDATE_CAL_STATUS_VALUE_INDEX];
                        update.selftest_status = buffer[NAVX_UPDATE_SELFTEST_STATUS_VALUE_INDEX];

                        return NAVX_UPDATE_MESSAGE_LENGTH;
                    }
                    return 0;
                }

                static int encodeNavXPosUpdate(char* protocol_buffer,
                                               float yaw, float pitch, float roll,
                                               float compass_heading, float altitude, float fused_heading,
                                               float linear_accel_x, float linear_accel_y, float linear_accel_z,
                                               float mpu_temp_c,
                                               int16_t quat_w, int16_t quat_x, int16_t quat_y, int16_t quat_z,
                                               float vel_x, float vel_y, float vel_z,
                                               float disp_x, float disp_y, float disp_z,
                                               uint8_t op_status, uint8_t sensor_status,
                                               uint8_t cal_status, uint8_t selftest_status)
                {
                    // Header
                    protocol_buffer[0] = PACKET_START_CHAR;
                    protocol_buffer[1] = BINARY_PACKET_INDICATOR_CHAR;
                    protocol_buffer[2] = NAVXPOS_UPDATE_MESSAGE_LENGTH - 2;
                    protocol_buffer[3] = MSGID_NAVXPOS_UPDATE;
                    // data
                    IMURegisters::encodeProtocolSignedHundredthsFloat(yaw, &protocol_buffer[NAVXPOS_UPDATE_YAW_VALUE_INDEX]);
                    IMURegisters::encodeProtocolSignedHundredthsFloat(pitch, &protocol_buffer[NAVXPOS_UPDATE_PITCH_VALUE_INDEX]);
                    IMURegisters::encodeProtocolSignedHundredthsFloat(roll, &protocol_buffer[NAVXPOS_UPDATE_ROLL_VALUE_INDEX]);
                    IMURegisters::encodeProtocolUnsignedHundredthsFloat(compass_heading, &protocol_buffer[NAVXPOS_UPDATE_HEADING_VALUE_INDEX]);
                    IMURegisters::encodeProtocol1616Float(altitude, &protocol_buffer[NAVXPOS_UPDATE_ALTITUDE_VALUE_INDEX]);
                    IMURegisters::encodeProtocolUnsignedHundredthsFloat(fused_heading, &protocol_buffer[NAVXPOS_UPDATE_FUSED_HEADING_VALUE_INDEX]);
                    IMURegisters::encodeProtocolSignedThousandthsFloat(linear_accel_x, &protocol_buffer[NAVXPOS_UPDATE_LINEAR_ACCEL_X_VALUE_INDEX]);
                    IMURegisters::encodeProtocolSignedThousandthsFloat(linear_accel_y, &protocol_buffer[NAVXPOS_UPDATE_LINEAR_ACCEL_Y_VALUE_INDEX]);
                    IMURegisters::encodeProtocolSignedThousandthsFloat(linear_accel_z, &protocol_buffer[NAVXPOS_UPDATE_LINEAR_ACCEL_Z_VALUE_INDEX]);
                    IMURegisters::encodeProtocol1616Float(vel_x, &protocol_buffer[NAVXPOS_UPDATE_VEL_X_VALUE_INDEX]);
                    IMURegisters::encodeProtocol1616Float(vel_y, &protocol_buffer[NAVXPOS_UPDATE_VEL_Y_VALUE_INDEX]);
                    IMURegisters::encodeProtocol1616Float(vel_z, &protocol_buffer[NAVXPOS_UPDATE_VEL_Z_VALUE_INDEX]);
                    IMURegisters::encodeProtocol1616Float(disp_x, &protocol_buffer[NAVXPOS_UPDATE_DISP_X_VALUE_INDEX]);
                    IMURegisters::encodeProtocol1616Float(disp_y, &protocol_buffer[NAVXPOS_UPDATE_DISP_Y_VALUE_INDEX]);
                    IMURegisters::encodeProtocol1616Float(disp_z, &protocol_buffer[NAVXPOS_UPDATE_DISP_Z_VALUE_INDEX]);
                    IMURegisters::encodeProtocolSignedHundredthsFloat(mpu_temp_c, &protocol_buffer[NAVXPOS_UPDATE_MPU_TEMP_VAUE_INDEX]);
                    IMURegisters::encodeProtocolInt16(quat_w, &protocol_buffer[NAVXPOS_UPDATE_QUAT_W_VALUE_INDEX]);
                    IMURegisters::encodeProtocolInt16(quat_x, &protocol_buffer[NAVXPOS_UPDATE_QUAT_X_VALUE_INDEX]);
                    IMURegisters::encodeProtocolInt16(quat_y, &protocol_buffer[NAVXPOS_UPDATE_QUAT_Y_VALUE_INDEX]);
                    IMURegisters::encodeProtocolInt16(quat_z, &protocol_buffer[NAVXPOS_UPDATE_QUAT_Z_VALUE_INDEX]);

                    protocol_buffer[NAVXPOS_UPDATE_OPSTATUS_VALUE_INDEX] = op_status;
                    protocol_buffer[NAVXPOS_UPDATE_SENSOR_STATUS_VALUE_INDEX] = sensor_status;
                    protocol_buffer[NAVXPOS_UPDATE_CAL_STATUS_VALUE_INDEX] = cal_status;
                    protocol_buffer[NAVXPOS_UPDATE_SELFTEST_STATUS_VALUE_INDEX] = selftest_status;
                    // Footer
                    encodeTermination(protocol_buffer, NAVXPOS_UPDATE_MESSAGE_LENGTH, NAVXPOS_UPDATE_MESSAGE_LENGTH - 4);
                    return NAVXPOS_UPDATE_MESSAGE_LENGTH;
                }

                static int decodeNavXPosUpdate(char* buffer, int length, struct NavXPosUpdate& update)
                {
                    if (length < NAVXPOS_UPDATE_MESSAGE_LENGTH) { return 0; }
                    if ((buffer[0] == PACKET_START_CHAR) &&
                            (buffer[1] == BINARY_PACKET_INDICATOR_CHAR) &&
                            (buffer[2] == NAVXPOS_UPDATE_MESSAGE_LENGTH - 2) &&
                            (buffer[3] == MSGID_NAVXPOS_UPDATE))
                    {

                        if (!verifyChecksum(buffer, NAVXPOS_UPDATE_MESSAGE_CHECKSUM_INDEX)) { return 0; }

                        update.yaw = IMURegisters::decodeProtocolSignedHundredthsFloat(&buffer[NAVXPOS_UPDATE_YAW_VALUE_INDEX]);
                        update.pitch = IMURegisters::decodeProtocolSignedHundredthsFloat(&buffer[NAVXPOS_UPDATE_PITCH_VALUE_INDEX]);
                        update.roll = IMURegisters::decodeProtocolSignedHundredthsFloat(&buffer[NAVXPOS_UPDATE_ROLL_VALUE_INDEX]);
                        update.compass_heading = IMURegisters::decodeProtocolUnsignedHundredthsFloat(&buffer[NAVXPOS_UPDATE_HEADING_VALUE_INDEX]);
                        update.altitude = IMURegisters::decodeProtocol1616Float(&buffer[NAVXPOS_UPDATE_ALTITUDE_VALUE_INDEX]);
                        update.fused_heading = IMURegisters::decodeProtocolUnsignedHundredthsFloat(&buffer[NAVXPOS_UPDATE_FUSED_HEADING_VALUE_INDEX]);
                        update.linear_accel_x = IMURegisters::decodeProtocolSignedThousandthsFloat(&buffer[NAVXPOS_UPDATE_LINEAR_ACCEL_X_VALUE_INDEX]);
                        update.linear_accel_y = IMURegisters::decodeProtocolSignedThousandthsFloat(&buffer[NAVXPOS_UPDATE_LINEAR_ACCEL_Y_VALUE_INDEX]);
                        update.linear_accel_z = IMURegisters::decodeProtocolSignedThousandthsFloat(&buffer[NAVXPOS_UPDATE_LINEAR_ACCEL_Z_VALUE_INDEX]);
                        update.vel_x = IMURegisters::decodeProtocol1616Float(&buffer[NAVXPOS_UPDATE_VEL_X_VALUE_INDEX]);
                        update.vel_y = IMURegisters::decodeProtocol1616Float(&buffer[NAVXPOS_UPDATE_VEL_Y_VALUE_INDEX]);
                        update.vel_z = IMURegisters::decodeProtocol1616Float(&buffer[NAVXPOS_UPDATE_VEL_Z_VALUE_INDEX]);
                        update.disp_x = IMURegisters::decodeProtocol1616Float(&buffer[NAVXPOS_UPDATE_DISP_X_VALUE_INDEX]);
                        update.disp_y = IMURegisters::decodeProtocol1616Float(&buffer[NAVXPOS_UPDATE_DISP_Y_VALUE_INDEX]);
                        update.disp_z = IMURegisters::decodeProtocol1616Float(&buffer[NAVXPOS_UPDATE_DISP_Z_VALUE_INDEX]);
                        update.mpu_temp = IMURegisters::decodeProtocolSignedHundredthsFloat(&buffer[NAVXPOS_UPDATE_MPU_TEMP_VAUE_INDEX]);
                        /* NAVXPosUpdate:  Quaternions are signed int(16-bit resolution); divide by 16384 to yield +/- 2 radians */
                        update.quat_w = ((float)IMURegisters::decodeProtocolInt16(&buffer[NAVXPOS_UPDATE_QUAT_W_VALUE_INDEX]) / 16384.0f);
                        update.quat_x = ((float)IMURegisters::decodeProtocolInt16(&buffer[NAVXPOS_UPDATE_QUAT_X_VALUE_INDEX]) / 16384.0f);
                        update.quat_y = ((float)IMURegisters::decodeProtocolInt16(&buffer[NAVXPOS_UPDATE_QUAT_Y_VALUE_INDEX]) / 16384.0f);
                        update.quat_z = ((float)IMURegisters::decodeProtocolInt16(&buffer[NAVXPOS_UPDATE_QUAT_Z_VALUE_INDEX]) / 16384.0f);
                        update.op_status = buffer[NAVXPOS_UPDATE_OPSTATUS_VALUE_INDEX];
                        update.sensor_status = buffer[NAVXPOS_UPDATE_SENSOR_STATUS_VALUE_INDEX];
                        update.cal_status = buffer[NAVXPOS_UPDATE_CAL_STATUS_VALUE_INDEX];
                        update.selftest_status = buffer[NAVXPOS_UPDATE_SELFTEST_STATUS_VALUE_INDEX];

                        return NAVXPOS_UPDATE_MESSAGE_LENGTH;
                    }
                    return 0;
                }

                static int encodeNavXPosTSUpdate(char* protocol_buffer,
                                                 float yaw, float pitch, float roll,
                                                 float compass_heading, float altitude, float fused_heading,
                                                 float linear_accel_x, float linear_accel_y, float linear_accel_z,
                                                 float mpu_temp_c,
                                                 float quat_w, float quat_x, float quat_y, float quat_z,
                                                 float vel_x, float vel_y, float vel_z,
                                                 float disp_x, float disp_y, float disp_z,
                                                 uint8_t op_status, uint8_t sensor_status,
                                                 uint8_t cal_status, uint8_t selftest_status,
                                                 uint32_t timestamp)
                {
                    // Header
                    protocol_buffer[0] = PACKET_START_CHAR;
                    protocol_buffer[1] = BINARY_PACKET_INDICATOR_CHAR;
                    protocol_buffer[2] = NAVXPOS_TS_UPDATE_MESSAGE_LENGTH - 2;
                    protocol_buffer[3] = MSGID_NAVXPOS_TS_UPDATE;

                    // data
                    IMURegisters::encodeProtocol1616Float(yaw, &protocol_buffer[NAVXPOS_TS_UPDATE_YAW_VALUE_INDEX]);
                    IMURegisters::encodeProtocol1616Float(pitch, &protocol_buffer[NAVXPOS_TS_UPDATE_PITCH_VALUE_INDEX]);
                    IMURegisters::encodeProtocol1616Float(roll, &protocol_buffer[NAVXPOS_TS_UPDATE_ROLL_VALUE_INDEX]);
                    IMURegisters::encodeProtocol1616Float(compass_heading, &protocol_buffer[NAVXPOS_TS_UPDATE_HEADING_VALUE_INDEX]);
                    IMURegisters::encodeProtocol1616Float(altitude, &protocol_buffer[NAVXPOS_TS_UPDATE_ALTITUDE_VALUE_INDEX]);
                    IMURegisters::encodeProtocol1616Float(fused_heading, &protocol_buffer[NAVXPOS_TS_UPDATE_FUSED_HEADING_VALUE_INDEX]);
                    IMURegisters::encodeProtocol1616Float(linear_accel_x, &protocol_buffer[NAVXPOS_TS_UPDATE_LINEAR_ACCEL_X_VALUE_INDEX]);
                    IMURegisters::encodeProtocol1616Float(linear_accel_y, &protocol_buffer[NAVXPOS_TS_UPDATE_LINEAR_ACCEL_Y_VALUE_INDEX]);
                    IMURegisters::encodeProtocol1616Float(linear_accel_z, &protocol_buffer[NAVXPOS_TS_UPDATE_LINEAR_ACCEL_Z_VALUE_INDEX]);
                    IMURegisters::encodeProtocol1616Float(vel_x, &protocol_buffer[NAVXPOS_TS_UPDATE_VEL_X_VALUE_INDEX]);
                    IMURegisters::encodeProtocol1616Float(vel_y, &protocol_buffer[NAVXPOS_TS_UPDATE_VEL_Y_VALUE_INDEX]);
                    IMURegisters::encodeProtocol1616Float(vel_z, &protocol_buffer[NAVXPOS_TS_UPDATE_VEL_Z_VALUE_INDEX]);
                    IMURegisters::encodeProtocol1616Float(disp_x, &protocol_buffer[NAVXPOS_TS_UPDATE_DISP_X_VALUE_INDEX]);
                    IMURegisters::encodeProtocol1616Float(disp_y, &protocol_buffer[NAVXPOS_TS_UPDATE_DISP_Y_VALUE_INDEX]);
                    IMURegisters::encodeProtocol1616Float(disp_z, &protocol_buffer[NAVXPOS_TS_UPDATE_DISP_Z_VALUE_INDEX]);
                    IMURegisters::encodeProtocolSignedHundredthsFloat(mpu_temp_c, &protocol_buffer[NAVXPOS_TS_UPDATE_MPU_TEMP_VAUE_INDEX]);
                    IMURegisters::encodeProtocol1616Float(quat_w, &protocol_buffer[NAVXPOS_TS_UPDATE_QUAT_W_VALUE_INDEX]);
                    IMURegisters::encodeProtocol1616Float(quat_x, &protocol_buffer[NAVXPOS_TS_UPDATE_QUAT_X_VALUE_INDEX]);
                    IMURegisters::encodeProtocol1616Float(quat_y, &protocol_buffer[NAVXPOS_TS_UPDATE_QUAT_Y_VALUE_INDEX]);
                    IMURegisters::encodeProtocol1616Float(quat_z, &protocol_buffer[NAVXPOS_TS_UPDATE_QUAT_Z_VALUE_INDEX]);

                    protocol_buffer[NAVXPOS_TS_UPDATE_OPSTATUS_VALUE_INDEX] = op_status;
                    protocol_buffer[NAVXPOS_TS_UPDATE_SENSOR_STATUS_VALUE_INDEX] = sensor_status;
                    protocol_buffer[NAVXPOS_TS_UPDATE_CAL_STATUS_VALUE_INDEX] = cal_status;
                    protocol_buffer[NAVXPOS_TS_UPDATE_SELFTEST_STATUS_VALUE_INDEX] = selftest_status;
                    IMURegisters::encodeProtocolInt32((int32_t)timestamp, &protocol_buffer[NAVXPOS_TS_UPDATE_TIMESTAMP_INDEX]);

                    // Footer
                    encodeTermination(protocol_buffer, NAVXPOS_TS_UPDATE_MESSAGE_LENGTH, NAVXPOS_TS_UPDATE_MESSAGE_LENGTH - 4);
                    return NAVXPOS_TS_UPDATE_MESSAGE_LENGTH;
                }

                static int decodeNavXPosTSUpdate(char* buffer, int length, struct NavXPosTSUpdate& update)
                {
                    if (length < NAVXPOS_TS_UPDATE_MESSAGE_LENGTH) { return 0; }
                    if ((buffer[0] == PACKET_START_CHAR) &&
                            (buffer[1] == BINARY_PACKET_INDICATOR_CHAR) &&
                            (buffer[2] == NAVXPOS_TS_UPDATE_MESSAGE_LENGTH - 2) &&
                            (buffer[3] == MSGID_NAVXPOS_TS_UPDATE))
                    {

                        if (!verifyChecksum(buffer, NAVXPOS_TS_UPDATE_MESSAGE_CHECKSUM_INDEX)) { return 0; }

                        update.yaw = IMURegisters::decodeProtocol1616Float(&buffer[NAVXPOS_TS_UPDATE_YAW_VALUE_INDEX]);
                        update.pitch = IMURegisters::decodeProtocol1616Float(&buffer[NAVXPOS_TS_UPDATE_PITCH_VALUE_INDEX]);
                        update.roll = IMURegisters::decodeProtocol1616Float(&buffer[NAVXPOS_TS_UPDATE_ROLL_VALUE_INDEX]);
                        update.compass_heading = IMURegisters::decodeProtocol1616Float(&buffer[NAVXPOS_TS_UPDATE_HEADING_VALUE_INDEX]);
                        update.altitude = IMURegisters::decodeProtocol1616Float(&buffer[NAVXPOS_TS_UPDATE_ALTITUDE_VALUE_INDEX]);
                        update.fused_heading = IMURegisters::decodeProtocol1616Float(&buffer[NAVXPOS_TS_UPDATE_FUSED_HEADING_VALUE_INDEX]);
                        update.linear_accel_x = IMURegisters::decodeProtocol1616Float(&buffer[NAVXPOS_TS_UPDATE_LINEAR_ACCEL_X_VALUE_INDEX]);
                        update.linear_accel_y = IMURegisters::decodeProtocol1616Float(&buffer[NAVXPOS_TS_UPDATE_LINEAR_ACCEL_Y_VALUE_INDEX]);
                        update.linear_accel_z = IMURegisters::decodeProtocol1616Float(&buffer[NAVXPOS_TS_UPDATE_LINEAR_ACCEL_Z_VALUE_INDEX]);
                        update.vel_x = IMURegisters::decodeProtocol1616Float(&buffer[NAVXPOS_TS_UPDATE_VEL_X_VALUE_INDEX]);
                        update.vel_y = IMURegisters::decodeProtocol1616Float(&buffer[NAVXPOS_TS_UPDATE_VEL_Y_VALUE_INDEX]);
                        update.vel_z = IMURegisters::decodeProtocol1616Float(&buffer[NAVXPOS_TS_UPDATE_VEL_Z_VALUE_INDEX]);
                        update.disp_x = IMURegisters::decodeProtocol1616Float(&buffer[NAVXPOS_TS_UPDATE_DISP_X_VALUE_INDEX]);
                        update.disp_y = IMURegisters::decodeProtocol1616Float(&buffer[NAVXPOS_TS_UPDATE_DISP_Y_VALUE_INDEX]);
                        update.disp_z = IMURegisters::decodeProtocol1616Float(&buffer[NAVXPOS_TS_UPDATE_DISP_Z_VALUE_INDEX]);
                        update.mpu_temp = IMURegisters::decodeProtocolSignedHundredthsFloat(&buffer[NAVXPOS_TS_UPDATE_MPU_TEMP_VAUE_INDEX]);
                        update.quat_w = IMURegisters::decodeProtocol1616Float(&buffer[NAVXPOS_TS_UPDATE_QUAT_W_VALUE_INDEX]) / 16384.0f;
                        update.quat_x = IMURegisters::decodeProtocol1616Float(&buffer[NAVXPOS_TS_UPDATE_QUAT_X_VALUE_INDEX]) / 16384.0f;
                        update.quat_y = IMURegisters::decodeProtocol1616Float(&buffer[NAVXPOS_TS_UPDATE_QUAT_Y_VALUE_INDEX]) / 16384.0f;
                        update.quat_z = IMURegisters::decodeProtocol1616Float(&buffer[NAVXPOS_TS_UPDATE_QUAT_Z_VALUE_INDEX]) / 16384.0f;
                        update.op_status = buffer[NAVXPOS_TS_UPDATE_OPSTATUS_VALUE_INDEX];
                        update.sensor_status = buffer[NAVXPOS_TS_UPDATE_SENSOR_STATUS_VALUE_INDEX];
                        update.cal_status = buffer[NAVXPOS_TS_UPDATE_CAL_STATUS_VALUE_INDEX];
                        update.selftest_status = buffer[NAVXPOS_TS_UPDATE_SELFTEST_STATUS_VALUE_INDEX];
                        update.timestamp = (uint32_t)IMURegisters::decodeProtocolInt32(&buffer[NAVXPOS_TS_UPDATE_TIMESTAMP_INDEX]);

                        return NAVXPOS_UPDATE_MESSAGE_LENGTH;
                    }
                    return 0;
                }

                static int encodeMagCalCommand(char* protocol_buffer, NAVX_DATA_ACTION action, int16_t* bias, float* matrix, float earth_mag_field_norm)
                {
                    // Header
                    protocol_buffer[0] = PACKET_START_CHAR;
                    protocol_buffer[1] = BINARY_PACKET_INDICATOR_CHAR;
                    protocol_buffer[2] = MAG_CAL_CMD_MESSAGE_LENGTH - 2;
                    protocol_buffer[3] = MSGID_MAG_CAL_CMD;

                    // Data
                    protocol_buffer[MAG_CAL_DATA_ACTION_VALUE_INDEX] = action;
                    for (int i = 0; i < 3; i++)
                    {
                        IMURegisters::encodeProtocolInt16(  bias[i],
                                                            &protocol_buffer[MAG_X_BIAS_VALUE_INDEX + (i * sizeof(int16_t))]);
                    }
                    for (int i = 0; i < 9; i++)
                    {
                        IMURegisters::encodeProtocol1616Float(matrix[i], &protocol_buffer[MAG_XFORM_1_1_VALUE_INDEX + (i * sizeof(s_1616_float))]);
                    }
                    IMURegisters::encodeProtocol1616Float(earth_mag_field_norm, &protocol_buffer[MAG_CAL_EARTH_MAG_FIELD_NORM_VALUE_INDEX]);
                    // Footer
                    encodeTermination(protocol_buffer, MAG_CAL_CMD_MESSAGE_LENGTH, MAG_CAL_CMD_MESSAGE_LENGTH - 4);
                    return MAG_CAL_CMD_MESSAGE_LENGTH;
                }

                static int decodeMagCalCommand(char* buffer, int length,
                                               NAVX_DATA_ACTION& action,
                                               int16_t* bias,
                                               float* matrix,
                                               float& earth_mag_field_norm)
                {
                    if (length < MAG_CAL_CMD_MESSAGE_LENGTH) { return 0; }
                    if ((buffer[0] == PACKET_START_CHAR) &&
                            (buffer[1] == BINARY_PACKET_INDICATOR_CHAR) &&
                            (buffer[2] == MAG_CAL_CMD_MESSAGE_LENGTH - 2) &&
                            (buffer[3] == MSGID_MAG_CAL_CMD))
                    {

                        if (!verifyChecksum(buffer, MAG_CAL_CMD_MESSAGE_CHECKSUM_INDEX)) { return 0; }

                        action = (NAVX_DATA_ACTION)buffer[MAG_CAL_DATA_ACTION_VALUE_INDEX];
                        for (int i = 0; i < 3; i++)
                        {
                            bias[i] = IMURegisters::decodeProtocolInt16(&buffer[MAG_X_BIAS_VALUE_INDEX + (i * sizeof(int16_t))]);
                        }
                        for (int i = 0; i < 9; i++)
                        {
                            matrix[i] = IMURegisters::decodeProtocol1616Float(&buffer[MAG_XFORM_1_1_VALUE_INDEX + (i * sizeof(s_1616_float))]);
                        }
                        earth_mag_field_norm = IMURegisters::decodeProtocol1616Float(&buffer[MAG_CAL_EARTH_MAG_FIELD_NORM_VALUE_INDEX]);
                        return MAG_CAL_CMD_MESSAGE_LENGTH;
                    }
                    return 0;
                }

                static int encodeDataSetResponse(char* protocol_buffer, NAVX_DATA_TYPE type, NAVX_TUNING_VAR_ID subtype, uint8_t status)
                {
                    // Header
                    protocol_buffer[0] = PACKET_START_CHAR;
                    protocol_buffer[1] = BINARY_PACKET_INDICATOR_CHAR;
                    protocol_buffer[2] = DATA_SET_RESPONSE_MESSAGE_LENGTH - 2;
                    protocol_buffer[3] = MSGID_DATA_SET_RESPONSE;
                    // Data
                    protocol_buffer[DATA_SET_RESPONSE_DATATYPE_VALUE_INDEX] = type;
                    protocol_buffer[DATA_SET_RESPONSE_VARID_VALUE_INDEX] = subtype;
                    protocol_buffer[DATA_SET_RESPONSE_STATUS_VALUE_INDEX] = status;
                    // Footer
                    encodeTermination(protocol_buffer, DATA_SET_RESPONSE_MESSAGE_LENGTH, DATA_SET_RESPONSE_MESSAGE_LENGTH - 4);
                    return DATA_SET_RESPONSE_MESSAGE_LENGTH;
                }

                static int decodeDataSetResponse(char* buffer, int length, NAVX_DATA_TYPE& type, NAVX_TUNING_VAR_ID& subtype, uint8_t& status)
                {
                    if (length < DATA_SET_RESPONSE_MESSAGE_LENGTH) { return 0; }
                    if ((buffer[0] == PACKET_START_CHAR) &&
                            (buffer[1] == BINARY_PACKET_INDICATOR_CHAR) &&
                            (buffer[2] == DATA_SET_RESPONSE_MESSAGE_LENGTH - 2) &&
                            (buffer[3] == MSGID_DATA_SET_RESPONSE))
                    {

                        if (!verifyChecksum(buffer, DATA_SET_RESPONSE_MESSAGE_CHECKSUM_INDEX)) { return 0; }

                        type = (NAVX_DATA_TYPE)buffer[DATA_SET_RESPONSE_DATATYPE_VALUE_INDEX];
                        subtype = (NAVX_TUNING_VAR_ID)buffer[DATA_SET_RESPONSE_VARID_VALUE_INDEX];
                        status = buffer[DATA_SET_RESPONSE_STATUS_VALUE_INDEX];
                        return DATA_SET_RESPONSE_MESSAGE_LENGTH;
                    }
                    return 0;
                }

                static int encodeDataGetRequest(char* protocol_buffer, NAVX_DATA_TYPE type, NAVX_TUNING_VAR_ID subtype)
                {
                    // Header
                    protocol_buffer[0] = PACKET_START_CHAR;
                    protocol_buffer[1] = BINARY_PACKET_INDICATOR_CHAR;
                    protocol_buffer[2] = DATA_REQUEST_MESSAGE_LENGTH - 2;
                    protocol_buffer[3] = MSGID_DATA_REQUEST;
                    // Data
                    protocol_buffer[DATA_REQUEST_DATATYPE_VALUE_INDEX] = type;
                    protocol_buffer[DATA_REQUEST_VARIABLEID_VALUE_INDEX] = subtype;
                    // Footer
                    encodeTermination(protocol_buffer, DATA_REQUEST_MESSAGE_LENGTH, DATA_REQUEST_MESSAGE_LENGTH - 4);
                    return DATA_REQUEST_MESSAGE_LENGTH;
                }

                static int decodeDataGetRequest(char* buffer, int length, NAVX_DATA_TYPE& type, NAVX_TUNING_VAR_ID& subtype)
                {
                    if (length < DATA_REQUEST_MESSAGE_LENGTH) { return 0; }
                    if ((buffer[0] == PACKET_START_CHAR) &&
                            (buffer[1] == BINARY_PACKET_INDICATOR_CHAR) &&
                            (buffer[2] == DATA_REQUEST_MESSAGE_LENGTH - 2) &&
                            (buffer[3] == MSGID_DATA_REQUEST))
                    {

                        if (!verifyChecksum(buffer, DATA_REQUEST_CHECKSUM_INDEX)) { return 0; }

                        type = (NAVX_DATA_TYPE)buffer[DATA_REQUEST_DATATYPE_VALUE_INDEX];
                        subtype = (NAVX_TUNING_VAR_ID)buffer[DATA_REQUEST_VARIABLEID_VALUE_INDEX];

                        return DATA_REQUEST_MESSAGE_LENGTH;
                    }
                    return 0;
                }

                static int encodeBoardIdentityResponse(char* protocol_buffer, uint8_t type, uint8_t fw_rev,
                                                       uint8_t fw_ver_major, uint8_t fw_ver_minor, uint16_t fw_revision,
                                                       uint8_t* unique_id)
                {
                    // Header
                    protocol_buffer[0] = PACKET_START_CHAR;
                    protocol_buffer[1] = BINARY_PACKET_INDICATOR_CHAR;
                    protocol_buffer[2] = BOARD_IDENTITY_RESPONSE_MESSAGE_LENGTH - 2;
                    protocol_buffer[3] = MSGID_BOARD_IDENTITY_RESPONSE;
                    // Data
                    protocol_buffer[BOARD_IDENTITY_BOARDTYPE_VALUE_INDEX] = type;
                    protocol_buffer[BOARD_IDENTITY_HWREV_VALUE_INDEX] = fw_rev;
                    protocol_buffer[BOARD_IDENTITY_FW_VER_MAJOR] = fw_ver_major;
                    protocol_buffer[BOARD_IDENTITY_FW_VER_MINOR] = fw_ver_minor;
                    IMURegisters::encodeProtocolUint16(fw_revision, &protocol_buffer[BOARD_IDENTITY_FW_VER_REVISION_VALUE_INDEX]);
                    for (int i = 0; i < 12; i++)
                    {
                        protocol_buffer[BOARD_IDENTITY_UNIQUE_ID_0 + i] = unique_id[i];
                    }
                    // Footer
                    encodeTermination(protocol_buffer, BOARD_IDENTITY_RESPONSE_MESSAGE_LENGTH, BOARD_IDENTITY_RESPONSE_MESSAGE_LENGTH - 4);
                    return BOARD_IDENTITY_RESPONSE_MESSAGE_LENGTH;
                }

                static int decodeBoardIdentityResponse(char* buffer, int length,  struct BoardID& update)
                {
                    if (length < BOARD_IDENTITY_RESPONSE_MESSAGE_LENGTH) { return 0; }
                    if ((buffer[0] == PACKET_START_CHAR) &&
                            (buffer[1] == BINARY_PACKET_INDICATOR_CHAR) &&
                            (buffer[2] == BOARD_IDENTITY_RESPONSE_MESSAGE_LENGTH - 2) &&
                            (buffer[3] == MSGID_BOARD_IDENTITY_RESPONSE))
                    {
                        if (!verifyChecksum(buffer, BOARD_IDENTITY_RESPONSE_CHECKSUM_INDEX)) { return 0; }

                        update.type = buffer[BOARD_IDENTITY_BOARDTYPE_VALUE_INDEX];
                        update.hw_rev = buffer[BOARD_IDENTITY_HWREV_VALUE_INDEX];
                        update.fw_ver_major = buffer[BOARD_IDENTITY_FW_VER_MAJOR];
                        update.fw_ver_minor = buffer[BOARD_IDENTITY_FW_VER_MINOR];
                        update.fw_revision = IMURegisters::decodeProtocolUint16(&buffer[BOARD_IDENTITY_FW_VER_REVISION_VALUE_INDEX]);
                        for (int i = 0; i < 12; i++)
                        {
                            update.unique_id[i] = buffer[BOARD_IDENTITY_UNIQUE_ID_0 + i];
                        }
                        return BOARD_IDENTITY_RESPONSE_MESSAGE_LENGTH;
                    }
                    return 0;
                }

            };
        }//navx
    }//navigation
}//rip
#endif // _IMU_PROTOCOL_H_
