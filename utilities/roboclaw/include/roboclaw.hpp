/*
 * The RIP License (Revision 0.3):
 * This software is available without warranty and without support.
 * Use at your own risk. Literally. It might delete your filesystem or
 * eat your cat. As long as you retain this notice, you can do whatever
 * you want with this. If we meet some day, you owe me a beer.
 *
 * Go Vols!
 *
 *  __    __  ________  __    __        _______   ______  _______
 * |  \  |  \|        \|  \  /  \      |       \ |      \|       \
 * | $$  | $$ \$$$$$$$$| $$ /  $$      | $$$$$$$\ \$$$$$$| $$$$$$$\
 * | $$  | $$   | $$   | $$/  $$       | $$__| $$  | $$  | $$__/ $$
 * | $$  | $$   | $$   | $$  $$        | $$    $$  | $$  | $$    $$
 * | $$  | $$   | $$   | $$$$$\        | $$$$$$$\  | $$  | $$$$$$$
 * | $$__/ $$   | $$   | $$ \$$\       | $$  | $$ _| $$_ | $$
 *  \$$    $$   | $$   | $$  \$$\      | $$  | $$|   $$ \| $$
 *   \$$$$$$     \$$    \$$   \$$       \$$   \$$ \$$$$$$ \$$
 */
#ifndef ROBOCLAW_HPP
#define ROBOCLAW_HPP

#include <inttypes.h>
#include <array>
#include <vector>
#include <tuple>

#include <json.hpp>

#include <units.hpp>

#include "exceptions.hpp"
#include "motor_dynamics.hpp"
#include "pid_parameters.hpp"
#include "config.hpp"
extern "C"
{
  #include "serial.h"
}

namespace rip
{
    namespace utilities
    {
        namespace roboclaw
        {
            /**
             * @class Roboclaw
             * @brief Class for interfacing with the RoboClaw motor controllers
             *
             * @link http://www.ionmc.com/
             * @link https://www.pololu.com/category/124/roboclaw-motor-controllers
             */
            class Roboclaw
            {
            public:
                enum class Motor
                {
                    kM1,
                    kM2
                }; // enum class Motor

                enum class Command : uint8_t
                {
                    kM1Forward                = 0,
                    kM1Backward               = 1,
                    kSetMinMB                 = 2,
                    kSetMaxMB                 = 3,
                    kM2Forward                = 4,
                    kM2Backward               = 5,
                    kM17Bit                   = 6,
                    kM27Bit                   = 7,
                    kMixedForward             = 8,
                    kMixedBackward            = 9,
                    kMixedRight               = 10,
                    kMixedLeft                = 11,
                    kMixedFb                  = 12,
                    kMixedLr                  = 13,
                    kGetM1Enc                 = 16,
                    kGetM2Enc                 = 17,
                    kGetM1Speed               = 18,
                    kGetM2Speed               = 19,
                    kResetEnc                 = 20,
                    kGetVersion               = 21,
                    kSetM1EncCount            = 22,
                    kSetM2EncCount            = 23,
                    kGetMBatt                 = 24,
                    kGetLBatt                 = 25,
                    kSetMinLB                 = 26,
                    kSetMaxLB                 = 27,
                    kSetM1PID                 = 28,
                    kSetM2PID                 = 29,
                    kGetM1ISpeed              = 30,
                    kGetM2ISpeed              = 31,
                    kM1Duty                   = 32,
                    kM2Duty                   = 33,
                    kMixedDuty                = 34,
                    kM1Speed                  = 35,
                    kM2Speed                  = 36,
                    kMixedSpeed               = 37,
                    kM1SpeedAccel             = 38,
                    kM2SpeedAccel             = 39,
                    kMixedSpeedAccel          = 40,
                    kM1SpeedDist              = 41,
                    kM2SpeedDist              = 42,
                    kMixedSpeedDist           = 43,
                    kM1SpeedAccelDist         = 44,
                    kM2SpeedAccelDist         = 45,
                    kMixedSpeedAccelDist      = 46,
                    kGetBuffers               = 47,
                    kGetPWMS                  = 48,
                    kGetCurrents              = 49,
                    kMixedSpeed2Accel         = 50,
                    kMixedSpeed2AccelDist     = 51,
                    kM1DutyAccel              = 52,
                    kM2DutyAccel              = 53,
                    kMixedDutyAccel           = 54,
                    kReadM1PID                = 55,
                    kReadM2PID                = 56,
                    kSetMainVoltages          = 57,
                    kSetLogicVoltages         = 58,
                    kGetMinMaxMainVoltages    = 59,
                    kGetMinMaxLogicVoltages   = 60,
                    kSetM1PosPID              = 61,
                    kSetM2PosPID              = 62,
                    kReadM1PosPID             = 63,
                    kReadM2PosPID             = 64,
                    kM1SpeedAccelDeccelPos    = 65,
                    kM2SpeedAccelDeccelPos    = 66,
                    kMixedSpeedAccelDeccelPos = 67,
                    kSetM1DefaultAccel        = 68,
                    kSetM2DefaultAccel        = 69,
                    kSetPinFunctions          = 74,
                    kGetPinFunctions          = 75,
                    kSetDeadband              = 76,
                    kGetDeadband              = 77,
                    kGetEncoders              = 78,
                    kGetISpeeds               = 79,
                    kRestoreDefaults          = 80,
                    kGetTemp                  = 82,
                    kGetTemp2                 = 83,  //!< Only valid on some models
                    kGetError                 = 90,
                    kGetEncoderMode           = 91,
                    kSetM1EncoderMode         = 92,
                    kSetM2EncoderMode         = 93,
                    kWriteNVM                 = 94,
                    kReadNVM                  = 95,   //!< Reloads values from Flash into Ram
                    kSetConfig                = 98,
                    kGetConfig                = 99,
                    kSetM1MaxCurrent          = 133,
                    kSetM2MaxCurrent          = 134,
                    kGetM1MaxCurrent          = 135,
                    kGetM2MaxCurrent          = 136,
                    kSetPWMMode               = 148,
                    kGetPWMMode               = 149,
                    kFlagBootLoader           = 255
                }; // enum class Command


                /**
                 * @brief Constructor
                 * @param json config
                 */

                Roboclaw(nlohmann::json config, bool test=1);
                /**
                 * @brief Destructor
                 *
                 */

                ~Roboclaw();
                /**
                * @brief validates json config passed to roboclaw
                *
                */
                void validateConfig(nlohmann::json testcfg);
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////// PWM /////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
                static const int16_t kFullSpeedForward = 32767;
                static const int16_t kThreeQuarterSpeedForward = 24575;
                static const int16_t kHalfSpeedForward = 16384;
                static const int16_t kQuarterSpeedForward = 8192;
                static const int16_t kStop = 0;
                static const int16_t kQuarterSpeedBackward = -8192;
                static const int16_t kHalfSpeedBackward = -16384;
                static const int16_t kThreeQuarterSpeedBackward = -24575;
                static const int16_t kFullSpeedBackward = -32767;

                /**
                 * @brief Drive motor forward or backward
                 *
                 * Send: [Address, 32 or 33, Duty(2 Bytes), CRC(2 bytes)]
                 *
                 * @param motor The motor to set the speed for
                 * @param speed The speed of the motor. range: -32767 to +32767 (e.g. +-100% duty)
                 *
                 * @exception CommandFailure Thrown if the command fails
                 * @exception OutOfRange Thrown if the voltage is out of range
                 */
                void printdaResponse(std::vector<uint8_t> m_last_cmd);
                void drive(Motor motor, int16_t speed);

                /**
                 * @brief Drive motor forward or backward
                 *
                 * Send: [Address, 34, DutyM1(2 Bytes), DutyM2(2 Bytes), CRC(2 bytes)]
                 *
                 * @param speed The speed of the motors. range: -32767 to +32767 (e.g. +-100% duty)
                 *
                 * @exception CommandFailure Thrown if the command fails
                 * @exception OutOfRange Thrown if the voltage is out of range
                 */
                void drive(int16_t speed);

//////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// Main Battery //////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
                /**
                 * @brief Sets the main battery (B- / B+) minimum and maximum voltage level. The motors
                 *        will stop if the voltage is above the maximum or below the minimum
                 *
                 * Send: [Address, 57, Min(2 bytes), Max(2bytes, CRC(2 bytes)]
                 * Receive: [0xFF]
                 *
                 * @param min The minimum voltage to set
                 * @param max The maximum voltage to set
                 *
                 * @exception CommandFailure Thrown if the command fails
                 * @exception OutOfRange Thrown if the voltage is out of range
                 */
                void setMainVoltages(units::Voltage min, units::Voltage max);

                /**
                 * @brief Read the main battery voltage level connected to B+ and B- terminals.
                 *
                 * Send: [Address, 25]
                 * Receive: [Value.Byte1, Value.Byte0, CRC(2 bytes)]
                 *
                 * @returns The main battery voltage level
                 *
                 * @exception CommandFailure Thrown if the command fails
                 */
                units::Voltage readMainBatteryVoltage();

                /**
                 * @brief Read the main battery minimum and maximum voltage
                 *
                 * Send: [Address, 59]
                 * Receive: [Min(2 bytes), Max(2 bytes), CRC(2 bytes)]
                 *
                 * @returns The minimum (indexed 0) and maximum (indexed 1) voltage for the main battery
                 *
                 * @exception CommandFailure Thrown if the command fails
                 */
                std::array<units::Voltage, 2> readMinMaxMainVoltages();

//////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// Logic Battery //////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
                /**
                 * @brief Sets logic input (LB- / LB+) minimum voltage level. The roboclaw will shutdown
                 *        if the voltage is above the maximum or below the minimum.
                 *
                 * Send: [Address, 58, Min(2 bytes), Max(2bytes, CRC(2 bytes)]
                 * Receive: [0xFF]
                 *
                 * @param min The minimum logic voltage
                 * @param max The maximum logic voltage
                 *
                 * @exception CommandFailure Thrown if the command fails
                 * @exception OutOfRange Thrown if the voltage is out of range
                 */
                void setLogicVoltages(units::Voltage min, units::Voltage max);

                /**
                 * @brief Read a logic battery voltage level connected to LB+ and LB- terminals.
                 *
                 * Send: [Address, 26, Value, CRC(2 bytes)]
                 * Receive: [0xFF]
                 *
                 * @returns The logic battery voltage level
                 *
                 * @exception CommandFailure Thrown if the command fails
                 */
                units::Voltage readLogicBatteryVoltage();

                /**
                 * @brief Read the logic battery minimum and maximum voltages
                 *
                 * Send: [Address, 60]
                 * Receive: [Min(2 bytes), Max(2 bytes), CRC(2 bytes)]
                 *
                 * @return The logic battery minimum and maximum voltages
                 *
                 * @exception CommandFailure Thrown if the command fails
                 */
                std::array<units::Voltage, 2> readMinMaxLogicVoltages();

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////// Encoders ////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

                /**
                 * @brief Read the encoder count in ticks
                 *
                 * Send: [Address, 16 or 17]
                 * Receive: [Enc1(4 bytes), Status, CRC(2 bytes)]
                 *
                 * @param motor Which motor to read the encoder of
                 *
                 * @returns The encoder count in ticks
                 */
                int64_t readEncoderRaw(Motor motor);

                /**
                 * @brief Read the encoder count in ticks for both encoders
                 * @return
                 *
                 * @note Uses 2 sequential reads in order to get direction (which isn't
                 *       provided by the command for requesting both)
                 */
                std::array<long, 2> readEncodersRaw();

                /**
                 * @brief Read the encoder value (as a distance) for a single encoder
                 *
                 * @param motor Which motor to read the encoder of
                 *
                 * @returns The encoder value
                 */
                units::Distance readEncoder(Motor motor);

                /**
                 * @brief Returns the encoder values (as a distance) for both encoders
                 *
                 * @returns The encoder values (as a distance) for both encoders
                 *
                 * @note Uses 2 sequential reads in order to get direction (which
                 *       is not given in the command that returns both)
                 */
                std::array<units::Distance, 2> readEncoders();

                /**
                 * @brief Sets the encoder value in ticks
                 *
                 * @param motor Which to motor to set the encoder of
                 * @param ticks The number of ticks to set the current encoder count to
                 */
                void setEncoderRaw(Motor motor, int ticks);

                /**
                 * @brief Set the encoder value using distance
                 *
                 * @param motor Which to motor to set the encoder of
                 * @param d The distance to set the current encoder to
                 */
                void setEncoder(Motor motor, units::Distance d);

                /**
                 * @brief Returns the encoder velocity in ticks per second for a single encoder
                 *
                 * @param motor Which motor to read from
                 *
                 * @returns The encoder velocity in ticks per second for a single encoder
                 */
                long readEncoderVelocityRaw(Motor motor);

                /**
                 * @brief Returns the encoder velocity (as a distance)
                 *
                 * @param motor Which motor to read from
                 *
                 * @returns The encoder velocity (as a distance)
                 */
                units::Velocity readEncoderVelocity(Motor motor);

                /**
                 * @brief Returns the encoder velocities in ticks per second for both encoders
                 *
                 * @returns The encoder velocities in ticks per second for both encoders
                 */
                std::array<long, 2> readEncodersVelocityRaw();

                /**
                 * @brief Returns the encoder velocities (using actual units) for both encoders
                 *
                 * @returns The encoder velocities (using actual units) for both encoders
                 */
                std::array<units::Velocity, 2> readEncodersVelocity();

                /**
                 * @brief Resets the encoders
                 */
                void resetEncoders();

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////// Version /////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
                /**
                 * @brief Read the roboclaw firmware version.
                 *
                 * Send: [Address, 21]
                 * Receive: [“RoboClaw 10.2A v4.1.11”,10,0, CRC(2 bytes)]
                 *
                 * @returns The firmware version
                 *
                 * @note The command will return up to 48 bytes. The return string includes the product
                 * name and firmware version.
                 *
                 * @exception CommandFailure Thrown if the command fails
                 */
                std::string readVersion();

//////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////// PID ///////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
                /**
                 * @brief Sets the Velocity PID parameters
                 *
                 * Send: [Address, 28 or 29, D(4 bytes), P(4 bytes), I(4 bytes), QPPS(4 byte), CRC(2 bytes)]
                 * Receive: [0xFF]
                 *
                 * @param motor Which motor to set the parameters for
                 * @param parameters The parameters used to control the motor
                 */
                void setVelocityPID(Motor motor, const VelocityPIDParameters& parameters);

                /**
                 * @brief Read the Velocity PID parameters
                 *
                 * @param motor Which motor to return the velocity PID parameters for
                 *
                 * @returns The velocity PID parameters for a single motor
                 */
                VelocityPIDParameters readVelocityPID(Motor motor);

                /**
                 * @brief setPositionPID
                 *
                 * Send: [Address, 61 or 62, D(4 bytes), P(4 bytes), I(4 bytes), MaxI(4 bytes), Deadzone(4 bytes), MinPos(4 bytes), MaxPos(4 bytes), CRC(2 bytes)]
                 * Receive: [0xFF]
                 *
                 * @param motor Which motor to set the position PID parameters for
                 *
                 * @param parameters The position PID parameters to set
                 */
                void setPositionPID(Motor motor, const PositionPIDParameters& parameters);

                /**
                 * @brief readPositionPID
                 *
                 * @param motor
                 *
                 * @returns
                 */
                PositionPIDParameters readPositionPID(Motor motor);

                /**
                 * @brief Sets the motor dynamics (speed, acceleration, deceleration, etc) for a single motor
                 *
                 * @param motor Which motor to set the dynamics for
                 * @param dynamics The dynamics to set
                 */
                void setDynamics(Motor motor, const MotorDynamics& dynamics, bool respectBuffer = true);

                /**
                 * @brief Sets the motor dynamics (speed, acceleration, decceleration, etc) for both motors
                 * @param dynamics The dynamics to set
                 */
                void setDynamics(const MotorDynamics& dynamics, bool respectBuffer = true);


//////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////// Misc //////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

                /**
                 * @brief Returns the current current draw for a specific motor
                 *
                 * @param motor Which motor
                 *
                 * @returns The current draw
                 */
                units::Current readCurrent(Motor motor);

                /**
                 * @brief Read the current draw from both motors.
                 *
                 * Send: [Address, 49]
                 * Receive: [M1 Current(2 bytes), M2 Currrent(2 bytes), CRC(2 bytes)]
                 *
                 * @return An array of 2 elements containing the current draw for both motors
                 */
                std::array<units::Current, 2> readCurrents();

                /**
                 * @brief Sets the maximum current the motor controller should allow for a single motor
                 *
                 * Send: [Address, 134, MaxCurrent(4 bytes), 0, 0, 0, 0, CRC(2 bytes)]
                 * Receive: [0xFF]
                 *
                 * @param motor Which motor
                 * @param current The current limit
                 */
                void setMaxCurrent(Motor motor, units::Current current);

                /**
                 * @brief readMaxCurrent
                 *
                 * Send: [Address, 135]
                 * Receive: [MaxCurrent(4 bytes), MinCurrent(4 bytes), CRC(2 bytes)]
                 *
                 * @param motor
                 * @return
                 */
                units::Current readMaxCurrent(Motor motor);

                /**
                 * @brief Read the board temperature
                 *
                 * Send: [Address, 82]
                 * Receive: [Temperature(2 bytes), CRC(2 bytes)]
                 *
                 * @returns The temperature of the board
                 */
                units::Temperature readTemperature();

                /*
                @brief: reads the number of commands in the buffer for
                both motors.
                Send: [Address, 47]
                Receive: [BufferM1, BufferM2, CRC(2 bytes)]

                @returns Buffer length for motors 1 and 2
                */
                std::vector<uint8_t> readBufferLens();
                /*
                @brief: reads the number of commands in the buffer for
                both motors, returns the buffer for just the specified motor.
                Send: [Address, 47]
                Receive: [BufferM1, BufferM2, CRC(2 bytes)]

                @returns Buffer length for motors 1 or 2
                */
                uint8_t readBufferLen(Motor motor);

                enum class S3Modes
                {
                    kDefault       = 0, //< Flip switch if in RC/Analog mode or E-Stop (latching) in Serial modes
                    kEStopLatching = 1, //< Causes the roboclaw to shutdown until the unit is power cycled
                    kEStop         = 2, //< Holds the roboclaw in shutdown until the E-Stop is cleared
                    kVoltageClamp  = 3  //< Sets the signal pin as an output to drive an external voltage clamp circuit
                };

                enum class S4Modes
                {
                    kDisabled      = 0, //< Pin is inactive
                    kEStopLatching = 1, //< Causes the roboclaw to shutdown until the unit is power cycled
                    kEStop         = 2, //< Holds the roboclaw in shutdown until the E-Stop is cleared
                    kVoltageClamp  = 3, //< Sets the signal pin as an output to drive an external voltage clamp circuit
                    kM1Home        = 4  //< Triggers the motor to sstop and the encoder count to reset to 0
                };

                enum class S5Modes
                {
                    kDisabled      = 0, //< Pin is inactive
                    kEStopLatching = 1, //< Causes the roboclaw to shutdown until the unit is power cycled
                    kEStop         = 2, //< Holds the roboclaw in shutdown until the E-Stop is cleared
                    kVoltageClamp  = 3, //< Sets the signal pin as an output to drive an external voltage clamp circuit
                    kM2Home        = 4  //< Triggers the motor to sstop and the encoder count to reset to 0
                };

                /**
                 * @brief setPinFunctions
                 *
                 * Send: [Address, 74, S3mode, S4mode, S5mode, CRC(2 bytes)]
                 * Receive: [0xFF]
                 *
                 * @param s3
                 * @param s4
                 * @param s5
                 * @return
                 */
                bool setPinModes(S3Modes s3, S4Modes s4, S5Modes s5);

                /**
                 * @brief getPinModes
                 *
                 * Send: [Address, 75]
                 * Receive: [S3mode, S4mode, S5mode, CRC(2 bytes)]
                 *
                 * @return
                 */
                std::tuple<S3Modes, S4Modes, S5Modes> getPinModes();

                enum class Status
                {
                    kNormal                 = 0x0000,
                    kM1OverCurrentWarning   = 0x0001,
                    kM2OverCurrentWarning   = 0x0002,
                    kEStop                  = 0x0004,
                    kM1TemperatureError     = 0x0008,
                    kM2TemperatureError     = 0x0010,
                    kMainBatteryHighError   = 0x0020,
                    kLogicBatteryHighError  = 0x0040,
                    kLogicBatteryLowError   = 0x0080,
                    kM1DriverFault          = 0x0100,
                    kM2DriverFault          = 0x0200,
                    kMainBatteryHighWarning = 0x0400,
                    kMainBatteryLowWarning  = 0x0800,
                    kM1TemperatureWarning   = 0x1000,
                    kM2TemperatureWarning   = 0x2000,
                    kM1Home                 = 0x4000,
                    kM2Home                 = 0x8000
                };

                /**
                 * @brief Returns the complete roboclaw status as an array of bools
                 * corresponding to each status in order.
                 * Send: [Address, 90]
                 * Receive: [Status, CRC(2 bytes)]
                 *
                 * @return
                 */
                std::array<bool,17> readStatus();
                /**
                 * @brief Returns the status of a specific status as a bool
                 *
                 * Send: [Address, 90]
                 * Receive: [Status, CRC(2 bytes)]
                 *
                 * @return
                 */
                bool readStatus(Status s);

                /*
                Returns 0xFF for crc check. Overrided when using mock to bypass CRC check.
                */
                virtual uint8_t returnFF();


                /**
                 * @brief Set the config for the roboclaw
                 *
                 * Send: [Address, 98, Config(2 bytes), CRC(2 bytes)]
                 * Receive: [0xFF]
                 *
                 * @param config
                 */
                void setConfig(Config config);

                /**
                 * @brief Returns the config for the roboclaw
                 *
                 * Send: [Address, 99]
                 * Receive: [Config(2 bytes), CRC(2 bytes)]
                 *
                 * @return The config
                 */
                Config getConfig();

            private:
                static const uint8_t kMaxRetries = 10;

                /**
                 * @brief Clear the checksum
                 */
                void crcClear();

                /**
                 * @brief Update CRC (Cyclic Redundancy Check)
                 *
                 * @param data Data to update the checksum with
                 */
                void crcUpdate (uint8_t data);
                /*
                Updates CRC
                */
                uint16_t crcGet();
//////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////// Serial /////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
                /* Wrappers for some of periphery's C serial methods.
                These are probably going to be phased out when we find a better
                home for them, but they can stay here for now <3 */
                virtual void write(serial_t *serial, std::vector<uint8_t> command, size_t len);

                virtual uint8_t read(serial_t *serial, units::Time timeout_ms);

                void open(serial_t *serial, std::string device, uint32_t baudrate);

                void open(serial_t *serial, std::string device,
                uint32_t baudrate, unsigned int databits,
                serial_parity_t parity, unsigned int stopbits,
                bool xonxoff, bool rtscts);

                void close(serial_t *serial);

                /**
                 * @brief Sends N bytes to the roboclaw
                 *
                 * @tparam Args The types of the arguments
                 *
                 * @param args The arguments
                 *
                 * @return Whether the message was successfully sent
                 */
                template<typename... Args>
                void writeN(Args... args)
                {
                    std::vector<uint8_t> command = {m_address};
                    argsToVector<0, Args...>(command, std::make_tuple(args...));
                    uint8_t data;
                    for (int try_ = 0; try_ < kMaxRetries; try_++)
                    {
                        crcClear();
                        for (int index = 0; index < command.size(); index++)
                        {
                            crcUpdate(command[index]);
                        }
                        uint16_t crc = crcGet();
                        command.push_back(static_cast<uint8_t>(crc >> 8));
                        command.push_back(static_cast<uint8_t>(crc));

                        write(&m_serial, command, command.size());
                        data = read(&m_serial, m_timeout);
                        std::cout << "writeN debugging: " << std::hex << (int)data << std::endl;
                        printdaResponse(command);
                        if(data == returnFF())
                        {
                            return;
                        }
                        command.resize(command.size() - 2);
                    }
                    throw CommandFailure();
                }

                /**
                 * @brief Converts a series of arguments that integer based into individual bytes that are added to a vector
                 *
                 * @tparam I Recursive index
                 * @tparam Args The types of the arguments
                 *
                 * @param command The vector to fill
                 * @param args The argument to add to the vector
                 */
                template<int I = 0, typename... Args>
                typename std::enable_if < I < sizeof...(Args), void>::type argsToVector(std::vector<uint8_t>& command, std::tuple<Args...> tup)
                {
                    auto value = std::get<I>(tup);
                    size_t size_v = sizeof(value);
                    for (int i_v = 0; i_v < size_v; i_v++)
                    {
                        command.push_back(value >> (8 * (size_v - 1 - i_v)));
                    }

                    argsToVector < I + 1, Args... > (command, tup);
                };

                /**
                 * @brief Ends the recursion
                 *
                 * @tparam I Recursive index
                 * @tparam Args The types of the arguments
                 *
                 * @param command The vector to fill
                 * @param args The argument to add to the vector
                 */
                template<int I = 0, typename... Args>
                typename std::enable_if<I == sizeof...(Args), void>::type argsToVector(std::vector<uint8_t>& command, std::tuple<Args...> args) {};

                /**
                 * @brief Read N bytes
                 *
                 * @param n The number of bytes to read
                 * @param cmd The read command to send to the roboclaw
                 *
                 * @return N byte response
                 */
                virtual std::vector<uint8_t> readN(uint8_t n, Command cmd);
                uint16_t m_crc;
                units::Time m_timeout;
                uint8_t m_address;
                double m_ticks_per_rev;
                units::Distance m_wheel_radius;
                //serial members
                const char* m_device;
                uint32_t m_baudrate;
                serial_t m_serial;
                //advanced
                unsigned int m_databits, m_stopbits;
                bool m_xonxoff, m_rtscts;
                serial_parity_t m_parity;
                //TODO: support advanced serial opening

            }; // class Roboclaw

            inline uint8_t operator >>(const Roboclaw::Command& cmd, size_t shift)
            {
                return static_cast<uint8_t>(cmd) >> shift;
            }
        } // namespace roboclaw
    }
}
#endif // ROBOCLAW_HPP
