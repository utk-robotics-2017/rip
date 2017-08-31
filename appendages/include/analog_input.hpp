#ifndef ANALOG_INPUT_HPP
#define ANALOG_INPUT_HPP

namespace appendages
{
    class AnalogInput : public Appendage
    {
        public:
            bool read();

        protected:
            std::shared_ptr<Appendage> create(const nlohmann::json& config, const std::map<std::string, int>& command_map, std::shared_ptr<cmdmessenger::Device> device);

        private:
            AnalogInput(const nlohmann::json& config, config std::map<std::string, int>& command_map, std::shared_ptr<cmdmessenger::Device> device);

            int m_read;
            int m_read_result;
    }; // class AnalogInput
} // namespace appendages

#endif // ANALOG_INPUT_HPP
