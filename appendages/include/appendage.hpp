#ifndef APPENDAGE_HPP
#define APPENDAGE_HPP

#include <string>
#include <map>
#include <memory>

#include <json.hpp>
#include <device.hpp>

namespace appendages
{
    /**
     * @class Appendage
     * @brief The base class for appendages.
     */
    class Appendage
    {
        public:
            /**
             * @brief Returns the label for the the appendage
             *
             * @returns The label for the appendage
             */
            std::string getLabel() const;

            /**
             * @brief Returns the type of the appendage
             *
             * @returns The type of the appendage
             */
            std::string getType() const;

            /**
             * @brief pure virtual function that forces each appendage to have a create function
             *
             * @param config The config created by ArduinoGen for this specific appendage
             * @param command_map The mapping of commands to their numeric counter parts
             * @param device The device that this appendage is connected to
             *
             * @returns A shared pointer to the appendage
             */
            virtual std::shared_ptr<Appendage> create(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<cmdmessenger::Device> device) = 0;

        protected:
            /**
             * @brief Constructor
             *
             * @param config The config created by ArduinoGen for this specific appendage
             *
             * @note Is protected so that it cannot be used directly. The AppendageFactory must be used
             */
            Appendage(const nlohmann::json& config, std:shared_ptr<cmdmessenger::Device> device);

            cmdmessenger::Command createCommand(const std::string& command_key, const std::map<std::string, int>& command_map, const std::string& parameter_string);

        private:
            std::shared_ptr<cmdmessenger::Device> m_device;
            std::string m_label;
            std::string m_type;
    };
}

#endif // APPENDAGE_HPP
