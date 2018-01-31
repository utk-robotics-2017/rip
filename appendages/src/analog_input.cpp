#include "appendages/analog_input.hpp"

#include <utility>
#include <tuple>
#include <memory>

#include <cmd_messenger.hpp>

using namespace rip::utilities;

namespace rip
{
    namespace appendages
    {
        AnalogInput::AnalogInput(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<cmdmessenger::Device> device)
            : Appendage(config, device)
            , m_read(createCommand("kAnalogInputRead", command_map, cmdmessenger::ArduinoCmdMessenger::makeArgumentString<typename cmdmessenger::ArduinoCmdMessenger::IntegerType>()))
            , m_read_result(createCommand("kAnalogInputReadResult", command_map, cmdmessenger::ArduinoCmdMessenger::makeArgumentString<typename cmdmessenger::ArduinoCmdMessenger::IntegerType>()))
        {
        }

        int AnalogInput::read()
        {
            cmdmessenger::ArduinoCmdMessenger messenger;
            messenger.send<cmdmessenger::ArduinoCmdMessenger::IntegerType>(m_device, m_read, m_id);
            return std::get<0>(messenger.receive<cmdmessenger::ArduinoCmdMessenger::IntegerType>(m_read_result));
        }

        void AnalogInput::stop()
        {

        }

        bool AnalogInput::diagnostic()
        {
            return true;
        }

        std::shared_ptr<Appendage> AnalogInput::create(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<cmdmessenger::Device> device)
        {
            return std::dynamic_pointer_cast<Appendage>(std::shared_ptr<AnalogInput>(new AnalogInput(config, command_map, device)));
        }
    }
}
