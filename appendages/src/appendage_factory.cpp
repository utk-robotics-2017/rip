#include "appendage_factory.hpp"

#include <fmt/format.h>

// Appendages
#include "digital_input.hpp"
#include "analog_input.hpp"

#include "appendages_exceptions.hpp"

namespace rip
{
    namespace appendages
    {
        std::shared_ptr<AppendageFactory> AppendageFactory::m_singleton = std::shared_ptr<AppendageFactory>(nullptr);

        std::shared_ptr<AppendageFactory> AppendageFactory::getInstance()
        {
            if (!m_singleton)
            {
                m_singleton = std::shared_ptr<AppendageFactory>(new AppendageFactory);
            }
            return m_singleton;
        }

        std::shared_ptr<Appendage> AppendageFactory::makeAppendage(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<cmdmessenger::Device> device)
        {
            if (config.find("type") == config.end())
            {
                throw AppendageWithoutType(fmt::format("appendage missing type"));
            }

            return m_constructors["type"](config, command_map, device);
        }

        void AppendageFactory::registerAppendage(const std::string& type, std::function<std::shared_ptr<Appendage>(const nlohmann::json&,
                                                                                          const std::map<std::string, int>&,
                                                                                          std::shared_ptr<cmdmessenger::Device>)
                                                 > constructor)
        {
            m_constructors[type] = constructor;
        }

        AppendageFactory::AppendageFactory()
        {
            registerAppendage("Digital Input", &DigitalInput::create);
            registerAppendage("Analog Input", &AnalogInput::create);
        }
    }
}
