#include "appendages/servo.hpp"

#include <utility>
#include <tuple>
#include <memory>

#include <cmd_messenger/cmd_messenger.hpp>

using namespace rip::utilities;

namespace rip
{
    namespace appendages
    {
        Servo::Servo(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<cmdmessenger::Device> device)
            : Appendage(config, device)
            , m_write(createCommand("kServoWrite", command_map, cmdmessenger::ArduinoCmdMessenger::makeArgumentString<typename utilities::cmdmessenger::ArduinoCmdMessenger::IntegerType, typename utilities::cmdmessenger::ArduinoCmdMessenger::IntegerType>()))
        {
        }

        void Servo::write(int value)
        {
            cmdmessenger::ArduinoCmdMessenger messenger;
            messenger.send<int>(m_device, m_write, m_id, value);
        }

        void Servo::stop()
        {
            // no stop command?
        }

        bool Servo::diagnostic()
        {
            // TODO figure out what this is
            return true;
        }

        std::shared_ptr<Appendage> Servo::create(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<cmdmessenger::Device> device)
        {
            return std::dynamic_pointer_cast<Appendage>(std::shared_ptr<Servo>(new Servo(config, command_map, device)));
        }
    }
}
