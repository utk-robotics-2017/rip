#include "appendages/digital_input.hpp"

#include <utility>
#include <tuple>
#include <memory>

#include <cmd_messenger/cmd_messenger.hpp>

namespace rip
{
    namespace appendages
    {
        DigitalInput::DigitalInput(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<cmdmessenger::Device> device)
            : Appendage(config, device)
            , m_read(createCommand("kReadDigitalInputRead", command_map, cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::IntegerType>()))
            , m_read_result(createCommand("kDigitalInputReadResult", command_map, cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::BooleanType>()))
        {
        }

        bool DigitalInput::read()
        {
            cmdmessenger::ArduinoCmdMessenger messenger;
            messenger.send<cmdmessenger::ArduinoCmdMessenger::IntegerType>(m_device, m_read, m_id);
            return std::get<0>(messenger.receive<cmdmessenger::ArduinoCmdMessenger::BooleanType>(m_read_result));
        }

        void DigitalInput::stop()
        {

        }

        bool DigitalInput::diagnostic()
        {
            // todo
            return true;
        }

        std::shared_ptr<Appendage> DigitalInput::create(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<cmdmessenger::Device> device)
        {
            return std::dynamic_pointer_cast<Appendage>(std::shared_ptr<DigitalInput>(new DigitalInput(config, command_map, device)));
        }
    }
}
