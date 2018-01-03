#ifndef APPENDAGE_TEMPLATE_HPP
#define APPENDAGE_TEMPLATE_HPP

#include <memory>
#include <vector>

#include "xml_element.hpp"
#include "includes.hpp"
#include "constructors.hpp"
#include "setup.hpp"
#include "loop.hpp"
#include "command.hpp"

namespace rip
{
    namespace arduinogen
    {
        /*
        class Includes;
        class Constructors;
        class Setup;
        class Loop;
        class Command;
        */

        class AppendageTemplate : private XmlElement
        {
        public:
            //AppendageTemplate();
            //~AppendageTemplate();

            //AppendageTemplate(const AppendageTemplate& other) = delete;
            //AppendageTemplate& operator=(const AppendageTemplate& other) = delete;

            AppendageTemplate(const tinyxml2::XMLElement* xml);

            std::shared_ptr<Includes> GetIncludes() const;

            std::shared_ptr<Constructors> GetConstructors() const;

            std::shared_ptr<Setup> GetSetup() const;

            std::shared_ptr<Loop> GetLoop() const;

            const std::vector<Command>& GetCommands() const;

        private:
            std::shared_ptr<Includes> m_includes;
            std::shared_ptr<Constructors> m_constructors;
            std::shared_ptr<Setup> m_setup;
            std::shared_ptr<Loop> m_loop;
            std::vector<Command> m_commands;
        };
    }
}

#endif //APPENDAGE_TEMPLATE_HPP
