#include "analog_input.hpp"

namespace rip
{
    namespace appendages
    {
        AnalogInput::AnalogInput(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<utilities::cmdmessenger::Device> device)
            : Appendage(std::forward<nlohmann::json>(config), device)
        {
            m_read = createCommand("kAnalogInputRead", command_map, "");
            m_read_result = createCommand("kAnalogInputReadResult", command_map, utilities::cmdmessenger::CmdMessenger::<int>());
        }

        bool DigitalInput::read()
        {
            utilities::cmdmessenger::CmdMessenger::send(m_device, m_read);
            return utilities::cmdmessenger::CmdMessenger::receive(m_device, m_read_result);
        }
    }
}