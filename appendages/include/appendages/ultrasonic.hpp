#ifndef ULTRASONIC_HPP
#define ULTRASONIC_HPP

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
        class Ultrasonic : public Appendage
        {
        public:
            /**
             * Reads the distance from the ultrasonic
             */
            units::Distance read();

            /**
             * Stop
             */
            virtual void stop() override;

            virtual bool diagnostic() override;

        protected:
            friend class AppendageFactory;

            /**
             * Function wrapper for the constructor so it can be pointed to
             *
             * @param config The config from arduino gen
             * @param command_map A map of the name of the commands to their enumerations
             * @param device The connection to the device
             */
            static std::shared_ptr<Appendage> create(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<cmdmessenger::Device> device);

        private:
            /**
             * Constructor
             *
             * @param config The config from arduino gen
             * @param command_map A map of the name of the commands to their enumerations
             * @param device The connection to the device
             */
            Ultrasonic(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<cmdmessenger::Device> device);

            std::shared_ptr<cmdmessenger::Command> m_read;
            std::shared_ptr<cmdmessenger::Command> m_read_result;
        };
    }
}
#endif
