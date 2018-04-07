#include "appendages/digital_output.hpp"

#include <utility>
#include <tuple>
#include <memory>

#include <cmd_messenger/cmd_messenger.hpp>

namespace rip
{
    namespace appendages
    {
        DigitalOutput::DigitalOutput(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<cmdmessenger::Device> device) 
            : Appendage(config, device)
            , m_write(createCommand("kWriteDigitalWrite", command_map, cmdmessenger::ArduinoCmdMessenger::makeArgumentString<cmdmessenger::ArduinoCmdMessenger::BooleanType>())
        {
        }

        void DigitalOutput::write(bool output)
        {
            cmdmessenger::ArduinoCmdMessenger messenger;
            messenger.send<cmdmessenger::ArduinoCmdMessenger::IntegerType, cmdmessenger::ArduinoCmdMessenger::BooleanType>(m_device, m_write, m_id, output);
        }

        void DigitalOutput::stop()
        {

        }

        bool DigitalOutput::diagnostic()
        {
            // todo
            return true;
        }

        std::shared_ptr<Appendage> DigitalOutput::create(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<cmdmessenger::Device> device)
        {
            return std::dynamic_pointer_cast<Appendage>(std::shared_ptr<DigitalOutput>(new DigitalOutput(config, command_map, device)));
        }

    }
}
