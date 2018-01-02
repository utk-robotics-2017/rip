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

        class AppendageTemplate : private XmlElement
        {
        public:
            AppendageTemplate();
            ~AppendageTemplate();

            AppendageTemplate(const tinyxml2::XMLElement* xml);

            std::unique_ptr<Includes> GetIncludes() const;

            std::unique_ptr<Constructors> GetConstructors() const;

            std::unique_ptr<Setup> GetSetup() const;

            std::unique_ptr<Loop> GetLoop() const;

            std::vector<Command> GetCommands() const;

        private:
            std::unique_ptr<Includes> m_includes;
            std::unique_ptr<Constructors> m_constructors;
            std::unique_ptr<Setup> m_setup;
            std::unique_ptr<Loop> m_loop;
            std::vector<Command> m_commands;
        };
    }
}

#endif //APPENDAGE_TEMPLATE_HPP
