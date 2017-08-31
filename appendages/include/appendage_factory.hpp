#ifndef APPENDAGE_FACTORY_HPP
#define APPENDAGE_FACTORY_HPP

namespace appendages
{
    class AppendageFactory
    {
        public:
            static std::shared_ptr<AppendageFactory> getInstance();

            std::shared_ptr<Appendage> makeAppendage(const nlohmann::json& config,
                                                     const std::map<std::string, int>&,
                                                     std::shared_ptr<cmdmessenger::Device>);
        private:
            static std::shared_ptr<AppendageFactory> m_singleton;

            AppendageFactory();

            void registerAppendage(std::string type,
                                   std::shared_ptr<Appendage> (*constructor)[](const nlohmann::json&,
                                                                               const std::map<std::string, int>&,
                                                                               std::shared_ptr<cmdmessenger::Device>));

            static std::map<std::string, std::shared_ptr<Appendage> [](const nlohmann::json&,
                    const std::map<std::string, int>&,
                    std::shared_ptr<cmdmessenger::Device>)> m_constructors;
    };
}
#endif // APPENDAGE_FACTORY_HPP
