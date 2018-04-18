#include "appendages/navx.hpp"
#include <utility>
#include <memory>
#include <cmd_messenger/cmd_messenger.hpp>
#include <chrono>
#include <thread>
#include <tuple>

namespace rip
{
    namespace appendages
    {
        NavX::NavX(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<cmdmessenger::Device> device)
                : Appendage(config, device), m_get_yaw(createCommand("kGetYaw", command_map, cmdmessenger::ArduinoCmdMessenger::makeArgumentString<>())), m_get_yaw_result(createCommand("kGetYawResult", command_map, cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::FloatType>())), m_get_pitch(createCommand("kGetPitch", command_map, cmdmessenger::ArduinoCmdMessenger::makeArgumentString<>())), m_get_pitch_result(createCommand("kGetPitchResult", command_map, cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::FloatType>())), m_get_roll(createCommand("kGetRoll", command_map, cmdmessenger::ArduinoCmdMessenger::makeArgumentString<>())), m_get_roll_result(createCommand("kGetRollResult", command_map, cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::FloatType>())), m_get_status(createCommand("kGetStatus", command_map, cmdmessenger::ArduinoCmdMessenger::makeArgumentString<>())), m_get_status_result(createCommand("kGetStatusResult", command_map, cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::CharType>()))
        {
        }

        std::shared_ptr<Appendage> NavX::create(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<cmdmessenger::Device> device)
        {
            return std::dynamic_pointer_cast<Appendage>(std::shared_ptr<NavX>(new NavX(config, command_map, device)));
        }

        units::Angle NavX::getYaw()
        {
            cmdmessenger::ArduinoCmdMessenger messenger;
            messenger.send<>(m_device, m_get_yaw);
            return std::get<0>(messenger.receive<cmdmessenger::ArduinoCmdMessenger::FloatType>(m_get_yaw_result)) * units::deg;
        }

        units::Angle NavX::getPitch()
        {
            cmdmessenger::ArduinoCmdMessenger messenger;
            messenger.send<>(m_device, m_get_pitch);
            return std::get<0>(messenger.receive<cmdmessenger::ArduinoCmdMessenger::FloatType>(m_get_pitch_result)) * units::deg;
        }

        units::Angle NavX::getRoll()
        {
            cmdmessenger::ArduinoCmdMessenger messenger;
            messenger.send<>(m_device, m_get_roll);
            return std::get<0>(messenger.receive<cmdmessenger::ArduinoCmdMessenger::FloatType>(m_get_roll_result)) * units::deg;
        }

        char NavX::getStatus()
        {
            cmdmessenger::ArduinoCmdMessenger messenger;
            messenger.send<>(m_device, m_get_status);
            return std::get<0>(messenger.receive<cmdmessenger::ArduinoCmdMessenger::CharType>(m_get_status_result));
        }

        void NavX::stop()
        {
            // Nothing stands in the way of the navx. The NavX cannot be stopped.
        }

        bool NavX::diagnostic()
        {
            misc::Logger::getInstance()->debug("Starting NavX appendage diagnostic utility");
            misc::Logger::getInstance()->debug("Reading yaw, pitch, roll for 5s");
            std::chrono::time_point<std::chrono::system_clock> start_time = std::chrono::system_clock::now();

            while(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start_time).count() < 5000)
            {
                misc::Logger::getInstance()->debug("Yaw: {} Pitch: {} Roll: {}", getYaw().to(units::deg), getPitch().to(units::deg), getRoll().to(units::deg));
                std::this_thread::sleep_for(std::chrono::milliseconds(125));
            }
            misc::Logger::getInstance()->debug("NavX appendage diagnostics complete");

            return true;
        }
    }
}
