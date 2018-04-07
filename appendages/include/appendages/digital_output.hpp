#ifndef DIGITAL_OUTPUT_HPP
#define DIGITAL_OUTPUT_HPP

#include <memory>
#include <map>
#include <string>

#include <cmd_messenger/command.hpp>

#include "appendages/appendage.hpp"
#include "appendages/appendage_factory.hpp"

#include <json.hpp>

namespace rip
{
    namespace appendages
    {
        class DigitalOutput : public Appendage
        {
            public:
            
                void write(bool output);

                virtual void stop() override;

                virtual bool diagnostic() override;

            protected:
                friend class AppendageFactory;

                static std::shared_ptr<Appendage> create(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<cmdmessenger::Device> device);

            private:

                DigitalOutput(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<cmd_messenger::Device> device);

                std::shared_ptr<cmdmessenger::Command> m_write;
        };
    }
}

#endif
