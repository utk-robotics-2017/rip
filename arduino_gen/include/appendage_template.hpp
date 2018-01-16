#ifndef APPENDAGE_TEMPLATE_HPP
#define APPENDAGE_TEMPLATE_HPP

#include <memory>
#include <vector>

#include "xml_element.hpp"

namespace rip
{
    namespace arduinogen
    {
        class Includes;
        class Constructors;
        class Setup;
        class Loop;
        class Command;
        class Appendage;

        class AppendageTemplate : private XmlElement
        {
        public:

            AppendageTemplate(const tinyxml2::XMLElement* xml, std::vector<std::shared_ptr<Appendage>> appendages);

            std::shared_ptr<Includes> GetIncludes() const;

            std::shared_ptr<Constructors> GetConstructors() const;

            std::shared_ptr<Setup> GetSetup() const;

            std::shared_ptr<Loop> GetLoop() const;

            const std::vector<std::shared_ptr<Command>>& GetCommands() const;

            const std::vector<std::shared_ptr<Appendage>>& GetAppendages() const;

        private:
            std::shared_ptr<Includes> m_includes;
            std::shared_ptr<Constructors> m_constructors;
            std::shared_ptr<Setup> m_setup;
            std::shared_ptr<Loop> m_loop;
            std::vector<std::shared_ptr<Command>> m_commands;
            std::vector<std::shared_ptr<Appendage>> m_appendages;
        };
    }
}

#endif //APPENDAGE_TEMPLATE_HPP
