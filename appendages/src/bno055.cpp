#include "appendages/bno055.hpp"

#include <cmd_messenger/cmd_messenger.hpp>

namespace rip
{
    namespace appendages
    {
        Bno055::Bno055(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<cmdmessenger::Device> device)
                : Appendage(config, device)
                , m_yaw(createCommand("kGetYaw", command_map, cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::IntegerType>()))
                , m_yaw_result(createCommand("kGetYawResult", command_map, cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::FloatType>()))
                , m_pitch(createCommand("kGetPitch", command_map, cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::IntegerType>()))
                , m_pitch_result(createCommand("kGetPitchResult", command_map, cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::FloatType>()))
                , m_roll(createCommand("kGetRoll", command_map, cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::IntegerType>()))
                , m_roll_result(createCommand("kGetRollResult", command_map, cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::FloatType>()))
                , m_calibrated(createCommand("kGetCalibrationStatus", command_map, cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::IntegerType>()))
                , m_caribrated_result(createCommand("kGetCalibrationStatusResult", command_map, cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::CharType>()))

        {
        }

        units::Angle Bno055::getYaw()
        {
            cmdmessenger::ArduinoCmdMessenger messenger;
            messenger.send<cmdmessenger::ArduinoCmdMessenger::IntegerType>(m_device, m_yaw, m_id);
            return std::get<0>(messenger.receive<cmdmessenger::ArduinoCmdMessenger::FloatType>(m_yaw_result)) * units::rad;
        }

        units::Angle Bno055::getPitch()
        {
            cmdmessenger::ArduinoCmdMessenger messenger;
            messenger.send<cmdmessenger::ArduinoCmdMessenger::IntegerType>(m_device, m_pitch, m_id);
            return std::get<0>(messenger.receive<cmdmessenger::ArduinoCmdMessenger::FloatType>(m_pitch_result)) * units::rad;
        }

        units::Angle Bno055::getRoll()
        {
            cmdmessenger::ArduinoCmdMessenger messenger;
            messenger.send<cmdmessenger::ArduinoCmdMessenger::IntegerType>(m_device, m_roll, m_id);
            return std::get<0>(messenger.receive<cmdmessenger::ArduinoCmdMessenger::FloatType>(m_roll_result)) * units::rad;
        }

        bool Bno055::isCalibrated()
        {
            cmdmessenger::ArduinoCmdMessenger messenger;
            messenger.send<cmdmessenger::ArduinoCmdMessenger::IntegerType>(m_device, m_calibrated, m_id);
            return std::get<0>(messenger.receive<cmdmessenger::ArduinoCmdMessenger::CharType>(m_caribrated_result));
        }

        double Bno055::get()
        {
            return getYaw()();
        }
    }
}