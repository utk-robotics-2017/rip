#ifndef APPENDAGE_FACTORY_HPP
#define APPENDAGE_FACTORY_HPP

#include <memory>
#include <map>
#include <string>
#include <functional>

#include <json.hpp>

#include "appendage.hpp"

namespace rip
{
    namespace appendages
    {
        /**
         * @class AppendageFactory
         * @brief
         */
        class AppendageFactory
        {
        public:
            /**
             * Returns the singleton to the AppendageFactory
             */
            static std::shared_ptr<AppendageFactory> getInstance();

            /**
             * Create an appendage from a config
             *
             * @param config The config for the appendage
             * @param command_map A map of command names to their enums
             * @param device The device that the appendage is attached to
             */
            std::shared_ptr<Appendage> makeAppendage(const nlohmann::json& config,
                    const std::map<std::string, int>& command_map,
                    std::shared_ptr<cmdmessenger::Device> device);
        private:
            static std::shared_ptr<AppendageFactory> m_singleton;

            /**
             * Constructor
             */
            AppendageFactory();

            /**
             * Register an appendage's constructor to be used for making instances of it
             *
             * @param type The type of the appendage
             * @param constructor A function pointer to a wrapper for the constructor of an Appendage derivative
             */
            void registerAppendage(const std::string& type,
                                   std::function<std::shared_ptr<Appendage>(const nlohmann::json&,
                                                                            const std::map<std::string, int>&,
                                                                            std::shared_ptr<cmdmessenger::Device>)
                                   > constructor
                                   );

            std::map<std::string, std::function<std::shared_ptr<Appendage>(const nlohmann::json&,
                                                                           const std::map<std::string, int>&,
                                                                           std::shared_ptr<cmdmessenger::Device>)
                                  >> m_constructors;
        };
    }
}
#endif // APPENDAGE_FACTORY_HPP
