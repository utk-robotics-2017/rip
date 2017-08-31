#include "analog_input.hpp"

namespace appendages
{
    AnalogInput::AnalogInput(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<cmdmessenger::Device> device)
        : Appendage(std::forward<nlohmann::json>(config), device)
    {
        m_read = createCommand("kAnalogInputRead", command_map, "");
        m_read_result = createCommand("kAnalogInputReadResult", command_map, cmdmessenger::CmdMessenger::<int>());
    }

    bool DigitalInput::read()
    {
        cmdmessenger::CmdMessenger::send(m_device, m_read);
        return cmdmessenger::CmdMessenger::receive(m_device, m_read_result);
    }
}
