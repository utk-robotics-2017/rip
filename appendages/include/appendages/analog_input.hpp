#ifndef ANALOG_INPUT_HPP
#define ANALOG_INPUT_HPP

#include <memory>
#include <map>
#include <string>

#include "appendages/appendage.hpp"

#include <cmd_messenger/device.hpp>
#include <cmd_messenger/command.hpp>

namespace rip
{
    namespace appendages
    {
        /**
         * @class AnalogInput
         *
         * @brief A simple analog input
         */
        class AnalogInput : public Appendage
        {
        public:
            /**
             * Reads the analog port
             * @returns The analog value [0 - 1023]
             */
            int read();

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
            static std::shared_ptr<Appendage> create(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<utilities::cmdmessenger::Device> device);

        private:
            /**
             * Constructor
             *
             * @param config The config from arduino gen
             * @param command_map A map of the name of the commands to their enumerations
             * @param device The connection to the device
             */
            AnalogInput(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<utilities::cmdmessenger::Device> device);

            std::shared_ptr<utilities::cmdmessenger::Command> m_read;
            std::shared_ptr<utilities::cmdmessenger::Command> m_read_result;
        }; // class AnalogInput
    } // namespace appendages
}
#endif // ANALOG_INPUT_HPP
