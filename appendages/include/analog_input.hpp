#ifndef ANALOG_INPUT_HPP
#define ANALOG_INPUT_HPP

#include <appendage.hpp>

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

        protected:
            std::shared_ptr<Appendage> create(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<utilities::cmdmessenger::Device> device) override;

        private:
            AnalogInput(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<utilities::cmdmessenger::Device> device);

            int m_read;
            int m_read_result;
        }; // class AnalogInput
    } // namespace appendages
}
#endif // ANALOG_INPUT_HPP
