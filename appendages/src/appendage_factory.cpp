#include "appendages/appendage_factory.hpp"

#include <fmt/format.h>

// Appendages
#include "appendages/digital_input.hpp"
#include "appendages/analog_input.hpp"
#include "appendages/roboclaw.hpp"
#include "appendages/servo.hpp"
#include "appendages/ultrasonic.hpp"

#include "appendages/exceptions.hpp"

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
            std::string appendage_type = config["type"];
            if (m_constructors.find(appendage_type) == m_constructors.end())
            {
                // we probably don't have this appendage type available to the factory
                throw AppendageNotImplemented(fmt::format("Factory: no Constructor for appendage type {} !", appendage_type));
            }
            misc::Logger::getInstance()->debug(fmt::format("Factory: Constructing an appendage of type {} ...", appendage_type));
            return m_constructors[appendage_type](config, command_map, device);
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
            registerAppendage("Roboclaw", &Roboclaw::create);
            registerAppendage("Servo", &Servo::create);
            registerAppendage("Ultrasonic", &Ultrasonic::create);
        }
    }
}
