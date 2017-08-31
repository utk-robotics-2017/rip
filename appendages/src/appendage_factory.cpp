#include "appendage_factory.hpp"

// Appendages
#include "digital_input.hpp"
#include "analog_input.hpp"

namespace appendages
{
    std::shared_ptr<AppendageFactory> AppendageFactory::m_singleton = std::shared_ptr<AppendageFactory>(nullptr);

    std::shared_ptr<AppendageFactory> AppendageFactory::getInstance()
    {
        if(!m_singleton)
        {
            m_singleton = std::make_shared<AppendageFactory>();
        }
        return m_singleton;
    }

    std::shared_ptr<Appendage> AppendageFactory::makeAppendage(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<cmdmessenger::Device> device)
    {
        if(config.find("type") == config.end())
        {
            throw AppendageWithoutType(fmt::format("{} appendage {} missing type", device->getName(), config.dump()));
        }

        return m_constructors["type"](std::forward<nlohmann::json>(config), std::forward< std::map<std::string, int> >(command_map), std::shared_ptr<cmdmessenger::Device>(device));
    }

    void registerAppendage(std::string type, std::shared_ptr<Appendage> (*constructor)[](const nlohmann::json&,
                                                                           const std::map<std::string, int>&,
                                                                           std::shared_ptr<cmdmessenger::Device>))
    {
        m_constructors[type] = constructor;
    }

    AppendageFactory::AppendageFactory()
    {
        registerAppendage("Digital Input", &DigitalInput::create);
        registerAppendage("Analog Input", &AnalogInput::create);
    }
}
