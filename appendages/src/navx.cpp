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
			: Appendage(config, device),
            m_get_yaw(createCommand("kGetYaw", command_map,
				cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::IntegerType>())),
            m_get_pitch(createCommand("kGetPitch", command_map,
				cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::IntegerType>())),
            m_get_roll(createCommand("kGetYaw", command_map,
				cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::IntegerType>())),
            m_get_status(createCommand("kGetStatus", command_map,
				cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::IntegerType>()))
        {}

        std::shared_ptr<Appendage> NavX::create(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<cmdmessenger::Device> device)
        {
            return std::dynamic_pointer_cast<Appendage>(std::shared_ptr<NavX>(new NavX(config, command_map, device)));
        }

        float NavX::getYaw()
        {
            cmdmessenger::ArduinoCmdMessenger messenger;
			messenger.send<cmdmessenger::ArduinoCmdMessenger::IntegerType>(m_device, m_get_yaw, m_id);
            return std::get<0>(messenger.receive<cmdmessenger::ArduinoCmdMessenger::FloatType>(m_get_yaw_result));
        }

        float NavX::getPitch()
        {
            cmdmessenger::ArduinoCmdMessenger messenger;
			messenger.send<cmdmessenger::ArduinoCmdMessenger::IntegerType>(m_device, m_get_pitch, m_id);
            return std::get<0>(messenger.receive<cmdmessenger::ArduinoCmdMessenger::FloatType>(m_get_pitch_result));
        }

        float NavX::getRoll()
        {
            cmdmessenger::ArduinoCmdMessenger messenger;
			messenger.send<cmdmessenger::ArduinoCmdMessenger::IntegerType>(m_device, m_get_roll, m_id);
            return std::get<0>(messenger.receive<cmdmessenger::ArduinoCmdMessenger::FloatType>(m_get_roll_result));
        }

        char NavX::getStatus()
        {
            cmdmessenger::ArduinoCmdMessenger messenger;
			messenger.send<cmdmessenger::ArduinoCmdMessenger::IntegerType>(m_device, m_get_status, m_id);
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
                std::cout << std::fixed << std::setprecision(2) << getYaw()<< "       " << getPitch() << "      " << getRoll() << "   " << '\r' << std::flush;
                std::this_thread::sleep_for(std::chrono::milliseconds(125));
            }
            misc::Logger::getInstance()->debug("NavX appendage diagnostics complete");
        }
    }
}
