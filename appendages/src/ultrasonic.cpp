#include "ultrasonic.hpp"

#include <utility>
#include <tuple>
#include <memory>

#include <cmd_messenger.hpp>

using namespace rip::utilities;

namespace rip
{
    namespace appendages
    {
        Ultrasonic::Ultrasonic(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<cmdmessenger::Device> device)
            : Appendage(config, device)
            , m_read(createCommand("kUltrasonicRead", command_map, cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::IntegerType>()))
            , m_read_result(createCommand("kUltrasonicReadResult", command_map, cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::UnsignedLongType>()))
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

        }

        bool Ultrasonic::diagnostic()
        {
            return true;
        }

        std::shared_ptr<Appendage> Ultrasonic::create(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<cmdmessenger::Device> device)
        {
            return std::dynamic_pointer_cast<Appendage>(std::shared_ptr<Ultrasonic>(new Ultrasonic(config, command_map, device)));
        }
    }
}
