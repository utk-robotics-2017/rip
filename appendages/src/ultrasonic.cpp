#include "appendages/ultrasonic.hpp"

#include <utility>
#include <tuple>
#include <memory>
#include <chrono>
#include <thread>

#include <misc/logger.hpp>

#include <cmd_messenger/cmd_messenger.hpp>

namespace rip
{
    namespace appendages
    {
        Ultrasonic::Ultrasonic(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<cmdmessenger::Device> device)
            : Appendage(config, device)
            , m_read(createCommand("kReadUltrasonic", command_map, cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::IntegerType>()))
            , m_read_result(createCommand("kReadUltrasonicResult", command_map, cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::UnsignedLongType>()))
        {
        }

        units::Distance Ultrasonic::read()
        {
            cmdmessenger::ArduinoCmdMessenger messenger;
            messenger.send<cmdmessenger::ArduinoCmdMessenger::IntegerType>(m_device, m_read, m_id);
            return std::get<0>(messenger.receive<cmdmessenger::ArduinoCmdMessenger::UnsignedLongType>(m_read_result)) * units::in;
        }

        void Ultrasonic::stop()
        {
            // don't need to do anything.
        }

        bool Ultrasonic::diagnostic()
        {
            std::chrono::time_point<std::chrono::system_clock> start_time = std::chrono::system_clock::now();
            misc::Logger::getInstance()->info("Reading the ultrasonic value for 10s.");

            while(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start_time).count() < 10000)
      			{
      				misc::Logger::getInstance()->info(fmt::format("Distance: {} cm", read().to(units::cm)));
      			}

            misc::Logger::getInstance()->info("Ultrasonic diag finished.");
            return true;
        }

        std::shared_ptr<Appendage> Ultrasonic::create(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<cmdmessenger::Device> device)
        {
            return std::dynamic_pointer_cast<Appendage>(std::shared_ptr<Ultrasonic>(new Ultrasonic(config, command_map, device)));
        }
    }
}
