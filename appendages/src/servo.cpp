#include "ultrasonic.hpp"

#include <utility>
#include <tuple>
#include <memory>

#include <cmd_messenger.hpp>

namespace rip
{
    namespace appendages
    {
        Servo::Servo(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<cmdmessenger::Device> device)
            : Appendage(config, device)
            , m_read(createCommand("kServoWrite", command_map, cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::IntegerType, cmdmessenger::ArduinoCmdMessenger::IntegerType>()))
        {
        }

        void Servo::write(int value)
        {
            cmdmessenger::ArduinoCmdMessenger messenger;
            messenger.send<int>(m_device, m_read, m_id, value);
        }

        void Servo::stop()
        {

        }

        bool Servo::diagnostic()
        {
            return true;
        }

        std::shared_ptr<Appendage> Servo::create(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<cmdmessenger::Device> device)
        {
            return std::dynamic_pointer_cast<Appendage>(std::shared_ptr<Servo>(new Servo(config, command_map, device)));
        }
    }
}
