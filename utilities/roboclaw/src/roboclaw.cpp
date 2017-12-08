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

#include "roboclaw.hpp"
#include <fmt/format.h>

namespace rip
{
    namespace utilities
    {
        namespace roboclaw
        {
            Roboclaw::Roboclaw(nlohmann::json config, bool test)
            {
                // todo(Andrew): check config for all values
                  Roboclaw::test = test;
                  if(Roboclaw::test) {
                    validateConfig(config);
                  }
                  else {
                  m_address = config["address"].get<uint8_t>();
                  m_timeout = config["timeout"];
                  m_ticks_per_rev = config["ticks_per_rev"];
                  m_wheel_radius = config["wheel_radius"];
                }
            }

            //////////////////////////////////////////////////////////////////////////////////////////////
            //////////////////////////////////////////// PWM /////////////////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////////////////
            void Roboclaw::drive(Motor motor, int16_t speed)
            {
                Command cmd;
                switch (motor)
                {
                    case Motor::kM1:
                        cmd = Command::kM1Duty;
                        break;
                    case Motor::kM2:
                        cmd = Command::kM2Duty;
                        break;
                }

                writeN(cmd, speed);

            }

            void Roboclaw::drive(int16_t speed)
            {
                writeN(Command::kMixedDuty, speed, speed);
            }

            //////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////////////////////// Main Battery //////////////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////////////////

            void Roboclaw::setMainVoltages(units::Voltage min, units::Voltage max)
            {
                if (min() < 6 || min() > 34)
                {
                    throw OutOfRange(fmt::format("Minimum main battery voltage of {} is out of the 6V-34V range", min()));
                }

                if (max() < 6 || max() > 34)
                {
                    throw OutOfRange(fmt::format("Maximum main battery voltage of {} is out of the 6V-34V range", max()));
                }
                if(min()>=max())
                {
                    throw OutOfRange(fmt::format("Minimum main battery voltage of {} should be less than the maximum main battery voltage of {}.", min(), max()));
                }

                uint16_t min_v = min() * 10;
                uint16_t max_v = max() * 10;
                writeN(Command::kSetMainVoltages, min_v, max_v);
            }

            units::Voltage Roboclaw::readMainBatteryVoltage()
            {
                std::vector<uint8_t> response = readN(2, Command::kGetMBatt);
                uint16_t v = static_cast<uint16_t>((response[0] << 8) + response[1]);
                return static_cast<double>(v) / 10.0 * units::V;
            }

            std::array<units::Voltage, 2> Roboclaw::readMinMaxMainVoltages()
            {
                std::array<units::Voltage, 2> rv;
                std::vector<uint8_t> response = readN(4, Command::kGetMinMaxMainVoltages);
                for (uint8_t i = 0; i < 2; i++)
                {
                    rv[1] += static_cast<double>(static_cast<uint16_t>(response[2 + i]) << (8 * (1 - i)));
                    rv[0] += static_cast<double>(static_cast<uint16_t>(response[i]) << (8 * (1 - i)));
                }
                rv[0] = rv[0]() / 10.0 * units::V;
                rv[1] = rv[1]() / 10.0 * units::V;
                return rv;
            }

            //////////////////////////////////////////////////////////////////////////////////////////////
            ///////////////////////////////////// Logic Battery //////////////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////////////////

            void Roboclaw::setLogicVoltages(units::Voltage min, units::Voltage max)
            {
                if (min() < 6 || min() > 34)
                {
                    throw OutOfRange(fmt::format("Minimum logic battery voltage of {} is out of the 6V-34V range", min()));
                }

                if (max() < 6 || max() > 34)
                {
                    throw OutOfRange(fmt::format("Maximum logic battery voltage of {} is out of the 6V-34V range", max()));
                }
                if(min()>=max())
                {
                    throw OutOfRange(fmt::format("Minimum logic battery voltage of {} should be less than the maximum logic battery voltage of {}.", min(), max()));
                }
                uint16_t min_v = min() * 10;
                uint16_t max_v = max() * 10;
                writeN(Command::kSetLogicVoltages, min_v, max_v);
            }

            units::Voltage Roboclaw::readLogicBatteryVoltage()
            {
                std::vector<uint8_t> response = readN(2, Command::kGetLBatt);
                uint16_t v = static_cast<uint16_t>((response[0] << 8) + response[1]);
                return static_cast<double>(v) / 10.0 * units::V;
            }

            std::array<units::Voltage, 2> Roboclaw::readMinMaxLogicVoltages()
            {
                std::array<units::Voltage, 2> rv;
                std::vector<uint8_t> response = readN(4, Command::kGetMinMaxLogicVoltages);
                for (uint8_t i = 0; i < 2; i++)
                {
                    rv[0] += response[2 + i] << (8 * (1 - i));
                    rv[1] += response[i] << (8 * (1 - i));
                }
                rv[0] = rv[0]() / 10.0 * units::V;
                rv[1] = rv[1]() / 10.0 * units::V;
                return rv;
            }

            //////////////////////////////////////////////////////////////////////////////////////////////
            //////////////////////////////////////// Encoders ////////////////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////////////////
            int64_t Roboclaw::readEncoderRaw(Motor motor)
            {
                Command cmd;
                switch (motor)
                {
                    case Motor::kM1:
                        cmd = Command::kGetM1Enc;
                        break;
                    case Motor::kM2:
                        cmd = Command::kGetM2Enc;
                        break;
                }
                std::vector<uint8_t> response = readN(5, cmd);

                int64_t rv=0;

                // First 4 bytes are the ticks per second
                rv = (static_cast<uint32_t>((response[0] << 8*3) + (response[1] << 8*2) + (response[2] << 8) + response[3]));

                // Status:
                // Bit0 - Counter Underflow (1= Underflow Occurred, Clear After Reading)
                // Bit1 - Direction (0 = Forward, 1 = Backward)
                // Bit2 - Counter Overflow (1 = Overflow Occurred, Clear After Reading)
                // Bit3-7 - Reserved
                if ((response[4] & 2)==2)
                {
                    rv *= -1;
                }
                return rv;
            }

            std::array<long, 2> Roboclaw::readEncodersRaw()
            {
                std::vector<uint8_t> response = readN(8, Command::kGetEncoders);
                std::array<long, 2> rv;
                rv[0] = static_cast<uint32_t>((response[0] << 8*3) + (response[1] << 8*2) + (response[2] << 8) + response[3]);
                rv[1] = static_cast<uint32_t>((response[4] << 8*3) + (response[5] << 8*2) + (response[6] << 8) + response[7]);
                return rv;
            }

            units::Distance Roboclaw::readEncoder(Motor motor)
            {
                long ticks = readEncoderRaw(motor);
                return (static_cast<double>(ticks) / m_ticks_per_rev) * m_wheel_radius() * (units::pi * 2);
            }

            std::array<units::Distance, 2> Roboclaw::readEncoders()
            {
                std::array<units::Distance, 2> ticks;
                ticks[0] = readEncoder(Motor::kM1);
                ticks[1] = readEncoder(Motor::kM2);
                return ticks;
            }
            /*
            Currently broken, sign conversions
            */
            void Roboclaw::setEncoderRaw(Motor motor, int ticks)
            {
                Command cmd;
                switch (motor)
                {
                    case Motor::kM1:
                        cmd = Command::kSetM1EncCount;
                        break;
                    case Motor::kM2:
                        cmd = Command::kSetM2EncCount;
                        break;
                }
                writeN(cmd, ticks);

            }

            void Roboclaw::setEncoder(Motor motor, units::Distance d)
            {
                setEncoderRaw(motor, d() * m_ticks_per_rev / m_wheel_radius() / (units::pi * 2));
            }

            long Roboclaw::readEncoderVelocityRaw(Motor motor)
            {
                // Pair of the count which has a range of 0 to 4,294,967,295 and status
                std::vector<uint8_t> response;
                Command cmd;
                switch (motor)
                {
                    case Motor::kM1:
                        cmd = Command::kGetM1Speed;
                        break;
                    case Motor::kM2:
                        cmd = Command::kGetM2Speed;
                        break;
                }

                response = readN(5, cmd);

                long rv;
                // First 4 bytes are the ticks per second
                rv = (static_cast<uint32_t>((response[0] << 8*3) + (response[1] << 8*2) + (response[2] << 8) + response[3]));

                // Status indicates the direction (0 – forward, 1 - backward).
                if ((response[4] & 2)==2)//used to be response[4]
                {
                    rv *= -1;
                }

                return rv;
            }

            units::Velocity Roboclaw::readEncoderVelocity(Motor motor)
            {
                return static_cast<double>(readEncoderVelocityRaw(motor)) / m_ticks_per_rev * m_wheel_radius() * (units::pi * 2);
            }

            std::array<long, 2> Roboclaw::readEncodersVelocityRaw()
            {
                std::array<long, 2> rv;
                rv[0] = readEncoderVelocityRaw(Motor::kM1);
                rv[1] = readEncoderVelocityRaw(Motor::kM2);
                return rv;
            }

            std::array<units::Velocity, 2> Roboclaw::readEncodersVelocity()
            {
                std::array<long, 2> ticks = readEncodersVelocityRaw();
                std::array<units::Velocity, 2> rv;
                rv[0] = ticks[0] / m_ticks_per_rev * m_wheel_radius() * (units::pi * 2);
                rv[1] = ticks[1] / m_ticks_per_rev * m_wheel_radius() * (units::pi * 2);
                return rv;
            }

            void Roboclaw::resetEncoders()
            {
                writeN(Command::kResetEnc);
            }

            //////////////////////////////////////////////////////////////////////////////////////////////
            //////////////////////////////////////// Version /////////////////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////////////////

            std::string Roboclaw::readVersion()
            {
                uint8_t data;
                std::string version;

                std::vector<uint8_t> command = {m_address, static_cast<uint8_t>(Command::kGetVersion)};

                for (uint8_t try_ = 0; try_ < kMaxRetries; try_++)
                {
                    data = 0;

                    crcClear();

                    for (uint8_t i = 0; i < command.size(); i++)
                    {
                        crcUpdate(command[i]);
                    }
                    write(command);

                    for (uint8_t i = 0; i < 48; i++)
                    {
                        if (data != -1)
                        {
                            data = read()[0];
                            version += data;
                            crcUpdate(version[i]);
                            if (version[i] == 0)
                            {
                                uint16_t ccrc;
                                data = read()[0];
                                if (data != -1)
                                {
                                    ccrc = static_cast<uint16_t>(data) << 8;
                                    data = read()[0];
                                    if (data != -1)
                                    {
                                        ccrc |= data;
                                        if (crcGet() == ccrc)
                                        {
                                            return version;
                                        }
                                        else
                                        {
                                            throw CommandFailure();
                                        }
                                    }
                                }
                                break;
                            }
                        }
                        else
                        {
                            break;
                        }
                    }
                }

                throw CommandFailure();
            }

            //////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////////////////////////// PID ///////////////////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////////////////

            void Roboclaw::setVelocityPID(Motor motor, const VelocityPIDParameters& parameters)
            {
                uint32_t kp = parameters.kp * 65536;
                uint32_t ki = parameters.ki * 65536;
                uint32_t kd = parameters.kd * 65536;
                if(parameters.initialized)
                {
                    Command cmd;
                    switch (motor)
                    {
                        case Motor::kM1:
                            cmd = Command::kSetM1PID;
                            break;
                        case Motor::kM2:
                            cmd = Command::kSetM2PID;
                            break;
                    }

                    writeN(cmd, kd, kp, ki, parameters.qpps);
                }
                else
                {
                    throw UninitializedStruct();
                }
            }

            VelocityPIDParameters Roboclaw::readVelocityPID(Motor motor)
            {
                Command cmd;
                switch (motor)
                {
                    case Motor::kM1:
                        cmd = Command::kReadM1PID;
                        break;
                    case Motor::kM2:
                        cmd = Command::kReadM2PID;
                        break;
                }

                std::vector<uint8_t> response = readN(16, cmd);

                VelocityPIDParameters v;
                uint32_t p=0, i=0, d=0;
                for (uint8_t index = 0; index < 4; index++)
                {
                    p |= response[index] << (8 * (3 - index));
                    i |= response[4 + index] << (8 * (3 - index));
                    d |= response[8 + index] << (8 * (3 - index));
                    v.qpps |= response[12 + index] << (8 * (3 - index));
                }
                v.kp = static_cast<float>(p) / 65536;
                v.ki = static_cast<float>(i) / 65536;
                v.kd = static_cast<float>(d) / 65536;
                v.initialized = true;
                return v;
            }

            void Roboclaw::setPositionPID(Motor motor, const PositionPIDParameters& parameters)
            {
                uint32_t kp = parameters.kp * 1024;
                uint32_t ki = parameters.ki * 1024;
                uint32_t kd = parameters.kd * 1024;
                if(parameters.initialized)
                {
                    Command cmd;
                    switch (motor)
                    {
                        case Motor::kM1:
                            cmd = Command::kSetM1PosPID;
                            break;
                        case Motor::kM2:
                            cmd = Command::kSetM2PosPID;
                            break;
                    }

                    writeN(cmd, kd, kp, ki, parameters.kiMax, parameters.deadzone, parameters.min, parameters.max);
                }
                else
                {
                    throw UninitializedStruct();
                }
            }

            PositionPIDParameters Roboclaw::readPositionPID(Motor motor)
            {
                Command cmd;
                switch (motor)
                {
                    case Motor::kM1:
                        cmd = Command::kReadM1PosPID;
                        break;
                    case Motor::kM2:
                        cmd = Command::kReadM2PosPID;
                        break;
                }

                std::vector<uint8_t> response = readN(28, cmd);
                PositionPIDParameters pid;
                uint32_t p, i, d;
                for (uint8_t index = 0; index < 4; index++)
                {
                    p |= response[index] << (8 * (3 - index));
                    i |= response[4 + index] << (8 * (3 - index));
                    d |= response[8 + index] << (8 * (3 - index));
                    pid.kiMax |= response[12 + index] << (8 * (3 - index));
                    pid.deadzone |= response[16 + index] << (8 * (3 - index));
                    pid.min |= response[20 + index] << (8 * (3 - index));
                    pid.max |= response[24 + index] << (8 * (3 - index));

                }
                pid.kp = static_cast<float>(p) / 1024;
                pid.ki = static_cast<float>(i) / 1024;
                pid.kd = static_cast<float>(d) / 1024;
                pid.initialized = true;
                return pid;
            }

            void Roboclaw::setDynamics(Motor motor, const MotorDynamics& dynamics, bool respectBuffer)
            {
                Command cmd;
                int32_t speed;
                uint32_t accel, dist, decel;

                switch (dynamics.getDType())
                {
                    case MotorDynamics::DType::kNone:
                        return;
                    case MotorDynamics::DType::kSpeed:
                        // Send: [Address, 35 or 36, Speed(4 Bytes), CRC(2 bytes)]
                        // Receive: [0xFF]
                        switch (motor)
                        {
                            case Motor::kM1:
                                cmd = Command::kM1Speed; //!< 35
                                break;
                            case Motor::kM2:
                                cmd = Command::kM2Speed; //!< 36
                                break;
                        }
                        speed = static_cast<int32_t>((*dynamics.getSpeed() / m_wheel_radius / (units::pi * 2))() * m_ticks_per_rev);
                        writeN(cmd, speed);
                        return;
                    case MotorDynamics::DType::kSpeedAccel:
                        // Send: [Address, 38 or 39, Accel(4 Bytes), Speed(4 Bytes), CRC(2 bytes)]
                        // Receive: [0xFF]
                        switch (motor)
                        {
                            case Motor::kM1:
                                cmd = Command::kM1SpeedAccel; //!< 38
                                break;
                            case Motor::kM2:
                                cmd = Command::kM2SpeedAccel; //!< 39
                                break;
                        }
                        speed = static_cast<int32_t>((*dynamics.getSpeed() / m_wheel_radius / (units::pi * 2))() * m_ticks_per_rev);
                        accel = static_cast<uint32_t>((*dynamics.getAcceleration() / m_wheel_radius / (units::pi * 2))() * m_ticks_per_rev);
                        writeN(cmd, accel, speed);
                        break;
                    case MotorDynamics::DType::kSpeedDist:
                        // Send: [Address, 41 or 42, Speed(4 Bytes), Distance(4 Bytes), Buffer, CRC(2 bytes)]
                        // Receive: [0xFF]
                        switch (motor)
                        {
                            case Motor::kM1:
                                cmd = Command::kM1SpeedDist; //!< 41
                                break;
                            case Motor::kM2:
                                cmd = Command::kM2SpeedDist; //!< 42
                                break;
                        }
                        speed = static_cast<int32_t>((*dynamics.getSpeed() / m_wheel_radius / (units::pi * 2))() * m_ticks_per_rev);
                        dist = static_cast<uint32_t>((*dynamics.getDistance() / m_wheel_radius / (units::pi * 2))() * m_ticks_per_rev);
                        writeN(cmd, speed, dist, static_cast<uint8_t>(respectBuffer));
                        break;
                    case MotorDynamics::DType::kSpeedAccelDist:
                        // Send: [Address, 44 or 45, Accel(4 bytes), Speed(4 Bytes), Distance(4 Bytes), Buffer, CRC(2 bytes)]
                        // Receive: [0xFF]
                        switch (motor)
                        {
                            case Motor::kM1:
                                cmd = Command::kM1SpeedAccelDist; //!< 44
                                break;
                            case Motor::kM2:
                                cmd = Command::kM2SpeedAccelDist; //!< 45
                                break;
                        }
                        speed = static_cast<int32_t>((*dynamics.getSpeed() / m_wheel_radius / (units::pi * 2))() * m_ticks_per_rev);
                        dist = static_cast<uint32_t>((*dynamics.getDistance() / m_wheel_radius / (units::pi * 2))() * m_ticks_per_rev);
                        accel = static_cast<uint32_t>((*dynamics.getAcceleration() / m_wheel_radius / (units::pi * 2))() * m_ticks_per_rev);

                        writeN(cmd, accel, speed, dist, static_cast<uint8_t>(respectBuffer));
                        break;
                    case MotorDynamics::DType::kSpeedAccelDecelDist:
                        // Send: [Address, 65, Accel(4 bytes), Speed(4 Bytes), Deccel(4 bytes), Position(4 Bytes), Buffer, CRC(2 bytes)]
                        // Receive: [0xFF]
                        switch (motor)
                        {
                            case Motor::kM1:
                                cmd = Command::kM1SpeedAccelDeccelPos; //!< 44
                                break;
                            case Motor::kM2:
                                cmd = Command::kM2SpeedAccelDeccelPos; //!< 45
                                break;
                        }
                        speed = static_cast<int32_t>((*dynamics.getSpeed() / m_wheel_radius / (units::pi * 2))() * m_ticks_per_rev);
                        dist = static_cast<uint32_t>((*dynamics.getDistance() / m_wheel_radius / (units::pi * 2))() * m_ticks_per_rev);
                        accel = static_cast<uint32_t>((*dynamics.getAcceleration() / m_wheel_radius / (units::pi * 2))() * m_ticks_per_rev);
                        decel = static_cast<uint32_t>((*dynamics.getDeceleration() / m_wheel_radius / (units::pi * 2))() * m_ticks_per_rev);
                        writeN(cmd, accel, speed, decel, dist, static_cast<uint8_t>(respectBuffer));
                        break;
                }
            }

            void Roboclaw::setDynamics(const MotorDynamics& dynamics, bool respectBuffer)
            {
                //todo(Andrew)
                return;
            }

            //////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////////////////////////// Misc //////////////////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////////////////

            std::array<units::Current, 2> Roboclaw::readCurrents()
            {
                std::array<units::Current, 2> rv;
                std::vector<uint8_t> response = readN(4, Command::kGetCurrents);
                for (uint8_t i = 0; i < 2; i++)
                {
                    rv[0] += response[2 + i] << (8 * (1 - i));
                    rv[1] += response[i] << (8 * (1 - i));
                }
                rv[0] = rv[0]() / 100.0 * units::A;
                rv[1] = rv[1]() / 100.0 * units::A;
                return rv;
            }

            units::Current Roboclaw::readCurrent(Motor motor)
            {
                std::array<units::Current, 2> rv = Roboclaw::readCurrents();
                switch (motor)
                {
                    case Motor::kM1:
                        return rv[0];
                        break;
                    case Motor::kM2:
                        return rv[1];
                        break;
                }
            }

            void Roboclaw::setMaxCurrent(Motor motor, units::Current current)
            {
                uint32_t value = current.to(units::A) * 100; // Convert to 10 mA
                Command cmd;
                switch (motor)
                {
                    case Motor::kM1:
                        cmd = Command::kSetM1MaxCurrent;
                        break;
                    case Motor::kM2:
                        cmd = Command::kSetM2MaxCurrent;
                        break;
                }
                writeN(cmd, value, 0); // Second value is a regular int (4 bytes) '0'
            }

            units::Current Roboclaw::readMaxCurrent(Motor motor)
            {
                Command cmd;
                switch (motor)
                {
                    case Motor::kM1:
                        cmd = Command::kGetM1MaxCurrent;
                        break;
                    case Motor::kM2:
                        cmd = Command::kGetM2MaxCurrent;
                        break;
                }
                std::vector<uint8_t> response = readN(4, cmd);
                // last 2 bytes are 0 because the minimum current is always 0
                return static_cast<double>(static_cast<uint32_t>((response[0] << 8*3) + (response[1] << 8*2) + (response[2] << 8) + response[3])) / 100.0 * units::A;

            }

            units::Temperature Roboclaw::readTemperature()
            {
                std::vector<uint8_t> response = readN(2, Command::kGetTemp);
                uint16_t value = static_cast<uint16_t>((response[0] << 8) + response[1]);
                return static_cast<double>(value) / 10.0 * units::degC; //todo(Andrew): Figure out the units
            }

            void Roboclaw::crcClear()
            {
                m_crc = 0;
            }

            uint8_t Roboclaw::returnFF()
            {
              return 0xFF;
            }

            void Roboclaw::crcUpdate(uint8_t data)
            {
                m_crc ^= (static_cast<uint16_t>(data) << 8);
                for (uint8_t i = 0; i < 8; i++)
                {
                    if (m_crc & 0x8000)
                    {
                        m_crc = (m_crc << 1) ^ 0x1021;
                    }
                    else
                    {
                        m_crc <<= 1;
                    }
                }
            }

            uint16_t Roboclaw::crcGet()
            {
                return m_crc;
            }

            std::vector<uint8_t> Roboclaw::readBufferLens()
            {
                /*
                Max number is 64. 128 if the buffer is empty.
                0 if the last command is being executed.
                */
                std::vector<uint8_t> response = readN(2, Command::kGetBuffers);
                return response;
            }
            uint8_t Roboclaw::readBufferLen(Motor motor)
            {
                /*
                Max number is 64. 128 if the buffer is empty.
                0 if the last command is being executed.
                */
                std::vector<uint8_t> response = readN(2, Command::kGetBuffers);
                switch (motor)
                {
                    case Motor::kM1:
                        return response[0];
                    case Motor::kM2:
                        return response[1];
                }
            }

            bool Roboclaw::readStatus(Roboclaw::Status s)
            {
                std::vector<uint8_t> response = readN(2, Command::kGetError);
                uint16_t value = static_cast<uint16_t>((response[0] << 8) + response[1]);
                uint16_t status = static_cast<uint16_t>(s);

                //ASSUMPTION MADE: If there is any error, status:normal is false.
                if(status == 0)
                {
                    if(value == 0)
                    {
                        //status normal
                        return true;
                    }
                    else
                    {
                        return false;
                    }
                }
                if((value & status) == status)
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }

            std::array<bool, 17> Roboclaw::readStatus()
            {
                //ASSUMPTION MADE: If there is any error, status:normal is false.
                std::vector<uint8_t> response = readN(2, Command::kGetError);
                uint16_t value = static_cast<uint16_t>((response[0] << 8) + response[1]);
                std::array<bool, 17> status;
                uint16_t mask = 1;
                if(value == 0)
                {
                    //status normal
                    status[0] = true;
                    for(int i=1; i<17; i++)
                    {
                        status[i] = false;
                    }
                    return status;
                }
                else
                {
                    status[0] = false;
                    for(int i=1; i<17; i++)
                    {
                        if((value & mask) == mask)
                        {
                            status[i] = true;
                        }
                        else
                        {
                            status[i] = false;
                        }
                        mask = (mask << 1);

                    }
                    return status;
                }
            }

            std::vector<uint8_t> Roboclaw::readN(uint8_t n, Command cmd)
            {
                uint8_t crc;

                uint8_t value = 0;
                uint8_t trys = kMaxRetries;
                int16_t data;

                std::vector<uint8_t> command = {m_address, static_cast<uint8_t>(cmd)};
                for (uint8_t try_ = 0; try_ < kMaxRetries; try_++)
                {
                    crcClear();
                    for (uint8_t i = 0; i < command.size(); i++)
                    {
                        crcUpdate(command[i]);
                    }

                    write(command);

                    std::vector<uint8_t> response;
                    uint8_t data;
                    for (uint8_t i = 0; i < n; i++)
                    {
                        data = read()[0];
                        crcUpdate(data);
                        response.push_back(data);
                        if (data == -1)
                        {
                            continue;
                        }
                    }

                    if (data != -1)
                    {
                        uint16_t ccrc;
                        data = read()[0];
                        if (data != -1)
                        {
                            ccrc = static_cast<uint16_t>(data) << 8;
                            data = read()[0];
                            if (data != -1)
                            {
                                ccrc |= data;
                                if (crcGet() == ccrc)
                                {
                                    return response;
                                }
                            }
                        }
                    }
                }

                throw ReadFailure();
            }
            void Roboclaw::validateConfig(nlohmann::json testcfg)
            {
              std::string vars[] = {"address", "timeout", "ticks_per_rev", "wheel_radius"};
              if(testcfg.empty()) {
                throw BadJson("JSON file was empty");
              }
              for(int i=0; i<4; i++) {
                if (testcfg.find(vars[i]) == testcfg.end()) {
                  throw BadJson(vars[i] + " was not found within json cfg.");
              }}
              if(testcfg.size()!= 4) {
                throw BadJson("Incorrect size json, size: " + std::to_string(testcfg.size()));
              }

              if((testcfg["address"] > 255) || (testcfg["address"] < 0))
              {
                throw OutOfRange("Address should be between 0 and 255");
              }
              for(int i=0; i<4; i++)
              {
                if(testcfg[vars[i]] < 0.0)
                {
                  throw OutOfRange(vars[i] + " should be a positive value");
                }
              }
              if(testcfg["wheel_radius"] == 0)
              {
                throw OutOfRange("wheel radius cannot = 0");
              }
              try {
                m_address = testcfg["address"].get<uint8_t>();
                m_timeout = testcfg["timeout"];
                m_ticks_per_rev = testcfg["ticks_per_rev"];
                m_wheel_radius = testcfg["wheel_radius"];
              }
              catch(...) {
                throw BadJson("Failed to set 1 or more values");
              }
            }
        }
    }
}
