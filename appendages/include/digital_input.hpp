#ifndef DIGITAL_INPUT_HPP
#define DIGITAL_INPUT_HPP

#include <appendage.hpp>

namespace rip
{
    namespace appendages
    {
        class DigitalInput : public Appendage
        {
        public:
            bool read();

        protected:
            std::shared_ptr<Appendage> create(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<cmdmessenger::Device> device);

        private:
            DigitalInput(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<cmdmessenger::Device> device);

            int m_read;
            int m_read_result;
        };
    }
}
#endif
