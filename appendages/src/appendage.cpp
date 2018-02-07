#include "appendages/appendage.hpp"

#include <fmt/format.h>
#include "appendages/exceptions.hpp"

using namespace rip::utilities;

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

        int Appendage::getId() const
        {
            return m_id;
        }

        std::shared_ptr<Appendage> Appendage::create(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<cmdmessenger::Device> device)
        {
            assert(false);
        }

        std::shared_ptr<cmdmessenger::Command> Appendage::createCommand(const std::string& command_key, const std::map<std::string, int>& command_map, const std::string& parameter_string)
        {
            if (command_map.find(command_key) == command_map.end())
            {
                throw CommandNotFound(fmt::format("Cannot find {} for appendage of type {}", command_key, m_type));
            }
            return std::make_shared<cmdmessenger::Command>(command_key, command_map.find(command_key)->second, parameter_string);
        }

        Appendage::Appendage(const nlohmann::json& config, std::shared_ptr<utilities::cmdmessenger::Device> device)
            : m_device(device)
        {

            if (config.find("type") == config.end())
            {
                throw AppendageWithoutType(fmt::format("appendage missing type"));
            }
            m_type = config["type"];

            if (config.find("label") == config.end())
            {
                throw AppendageWithoutLabel(fmt::format("appendage missing label"));
            }
            m_label = config["label"];

            if(config.find("id") == config.end())
            {
                throw AppendageWithId(fmt::format("appendage missing id"));
            }
            m_id = config["id"];
        }
    }
}
