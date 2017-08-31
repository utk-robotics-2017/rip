#include "digital_input.hpp"

namespace appendages
{
    DigitalInput::DigitalInput(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<cmdmessenger::Device> device)
        : Appendage(std::forward<nlohmann::json>(config), device)
    {
        m_read = createCommand("kDigitalInputRead", command_map, "");
        m_read_result = createCommand("kDigitalInputReadResult", command_map, cmdmessenger::CmdMessenger::<bool>());
    }

    bool DigitalInput::read()
    {
        cmdmessenger::CmdMessenger::send(m_device, m_read);
        return cmdmessenger::CmdMessenger::receive(m_device, m_read_result);
    }

    std::shared_ptr<Appendage> DigitalInput::create()
    {
        return dynamic_pointer_cast<Appendage>(std::make_shared<DigitalInput>(std::forward<nlohmann::json>(config), std::forward< std::map< std::string, int> >(command_map), std::forward< std::shared_ptr<cmdmessenger::Device> >(device)));
    }
}
