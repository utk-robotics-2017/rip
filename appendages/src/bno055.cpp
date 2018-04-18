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
            return std::get<0>(messenger.receive<cmdmessenger::ArduinoCmdMessenger::IntegerType>(m_yaw_result)) * units::deg;
        }

        units::Angle Bno055::getPitch()
        {
            cmdmessenger::ArduinoCmdMessenger messenger;
            messenger.send<cmdmessenger::ArduinoCmdMessenger::IntegerType>(m_device, m_pitch, m_id);
            return std::get<0>(messenger.receive<cmdmessenger::ArduinoCmdMessenger::FloatType>(m_pitch_result)) * units::deg;
        }

        units::Angle Bno055::getRoll()
        {
            cmdmessenger::ArduinoCmdMessenger messenger;
            messenger.send<cmdmessenger::ArduinoCmdMessenger::IntegerType>(m_device, m_roll, m_id);
            return std::get<0>(messenger.receive<cmdmessenger::ArduinoCmdMessenger::FloatType>(m_roll_result)) * units::deg;
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

        void Bno055::stop()
        {
        }

        bool Bno055::diagnostic()
        {
            misc::Logger::getInstance()->debug("Starting IMU appendage diagnostic utility");
            misc::Logger::getInstance()->debug("Reading yaw, pitch, roll for 5s");
            std::chrono::time_point<std::chrono::system_clock> start_time = std::chrono::system_clock::now();

            while(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start_time).count() < 5000)
            {
                misc::Logger::getInstance()->debug("Yaw: {} Pitch: {} Roll: {}", getYaw().to(units::deg), getPitch().to(units::deg), getRoll().to(units::deg));
                std::this_thread::sleep_for(std::chrono::milliseconds(125));
            }
            misc::Logger::getInstance()->debug("IMU appendage diagnostics complete");

            return true;
        }

        std::shared_ptr<Appendage> Bno055::create(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<cmdmessenger::Device> device)
        {
            return std::dynamic_pointer_cast<Appendage>(std::shared_ptr<Bno055>(new Bno055(config, command_map, device)));
        }
    }
}