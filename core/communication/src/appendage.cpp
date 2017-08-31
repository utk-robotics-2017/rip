#include "appendage.hpp"

namespace rip
{
    std::shared_ptr<Appendage> Appendage::makeAppendage(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<cmdmessenger::Device> device)
    {
        if(config.find("type") == config.end())
        {
            throw AppendageWithoutType(fmt::format("{} appendage {} missing type", device->getName(), config.dump()));
        }

        return m_constructors["type"](std::forward<nlohmann::json>(config, std::forward< std::map<std::string, int> >(command_map), std::shared_ptr<cmdmessenger::Device>(device)));
    }

    std::string Appendage::getLabel() const
    {
        return m_label;
    }

    std::string Appendage::getType() const
    {
        return m_type;
    }

    Appendage::Appendage(const nlohmann::json& config)
    {

        if(config.find("type") == config.end())
        {
            throw AppendageWithoutType(fmt::format("{} appendage {} missing type", device->getName(), config.dump()));
        }
        m_type = config["type"];

        if(config.find("label") == config.end())
        {
            throw AppendageWithoutLabel(fmt::format("{} appendage {} missing label", device->getName(), config.dump()));
        }

        m_label = config["label"];
    }

    void registerAppendage(std::string type, std::shared_ptr<Appendage> (*constructor)[](const nlohmann::json&,
                                                                           const std::map<std::string, int>&,
                                                                           std::shared_ptr<cmdmessenger::Device>))
    {
        m_constructors[type] = constructor;
    }
