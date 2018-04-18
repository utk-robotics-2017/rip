#ifndef IR_2018_HPP
#define IR_2018_HPP

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
        class Ir2018 : public Appendage
        {
        public:
            /**
             * Reads the ir receiver for the 2018 challenge
             * @returns Whether A, B, and C are right or left
             */
            std::array<bool, 5> read();

            bool calibrate();

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
            Ir2018(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<cmdmessenger::Device> device);

            std::shared_ptr<cmdmessenger::Command> m_read;
            std::shared_ptr<cmdmessenger::Command> m_read_result;
            std::shared_ptr<cmdmessenger::Command> m_calibrate;
            std::shared_ptr<cmdmessenger::Command> m_calibrate_result;
        };
    }
}
#endif // IR_2018_HPP
