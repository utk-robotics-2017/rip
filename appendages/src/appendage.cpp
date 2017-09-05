#include "appendage.hpp"

namespace rip
{
    namespace appendages
    {
        std::string Appendage::getLabel() const
        {
            return m_label;
        }

        std::string Appendage::getType() const
        {
            return m_type;
        }

        cmdmessenger::Command Appendage::createCommand(const std::string& command_key, const std::map<std::string, int>& command_map, const std::string& parameter_string)
        {
            if (command_map.find(command_key) == command_map.end())
            {
                throw CommandNotFound(fmt::format("Cannot find {} for appendage of type {}", command_key, m_type));
            }
            return cmdmessenger::Command(command_key, command_map[command_key], parameter_string);
        }

        Appendage::Appendage(const nlohmann::json& config, std::shared_ptr<cmdmessenger::Device> device)
            : m_device(device)
        {

            if (config.find("type") == config.end())
            {
                throw AppendageWithoutType(fmt::format("{} appendage {} missing type", device->getName(), config.dump()));
            }
            m_type = config["type"];

            if (config.find("label") == config.end())
            {
                throw AppendageWithoutLabel(fmt::format("{} appendage {} missing label", device->getName(), config.dump()));
            }
            m_label = config["label"];
        }
    }
}