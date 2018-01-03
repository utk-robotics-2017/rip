#include "appendage_template.hpp"

#include <fmt/format.h>
#include <tinyxml2.h>

#include "includes.hpp"
#include "constructors.hpp"
#include "setup.hpp"
#include "loop.hpp"
#include "command.hpp"
#include "exceptions.hpp"

namespace rip
{
    namespace arduinogen
    {
        //AppendageTemplate::AppendageTemplate() = default;
        //AppendageTemplate::~AppendageTemplate() = default;

        AppendageTemplate::AppendageTemplate(const tinyxml2::XMLElement* xml)
            : XmlElement(xml)
        {
            std::vector<const tinyxml2::XMLElement*> include_elements = getElements("includes");
            if (include_elements.size() == 1)
            {
                //m_includes = std::unique_ptr<Includes>(new Includes(include_elements[0]));
                m_includes = std::make_shared<Includes>(include_elements[0]);
            }
            else if (include_elements.size() > 1)
            {
                throw ElementException(fmt::format("AppendageTemplate can only have 1 includes element. Line number {}",
                                                    include_elements[1]->GetLineNum()));
            }

            std::vector<const tinyxml2::XMLElement*> constructor_elements = getElements("constructors");
            if (constructor_elements.size() == 1)
            {
                //m_constructors = std::unique_ptr<Constructors>(new Constructors(constructor_elements[0]));
                m_constructors = std::make_shared<Constructors>(constructor_elements[0]);
            }
            else if (constructor_elements.size() > 1)
            {
                throw ElementException(fmt::format("AppendageTemplate can only have 1 constructors element. Line number {}",
                                                    constructor_elements[1]->GetLineNum()));
            }

            std::vector<const tinyxml2::XMLElement*> setup_elements = getElements("setup");
            if (setup_elements.size() == 1)
            {
                //m_setup = std::unique_ptr<Setup>(new Setup(setup_elements[0]));
                m_setup = std::make_shared<Setup>(setup_elements[0]);
            }
            else if (setup_elements.size() > 1)
            {
                throw ElementException(fmt::format("AppendageTemplate can only have 1 setup element. Line number {}",
                                                    setup_elements[1]->GetLineNum()));
            }

            std::vector<const tinyxml2::XMLElement*> loop_elements = getElements("loop");
            if (loop_elements.size() == 1)
            {
                //m_loop = std::unique_ptr<Loop>(new Loop(loop_elements[0]));
                m_loop = std::make_shared<Loop>(loop_elements[0]);
            }
            else if (loop_elements.size() > 1)
            {
                throw ElementException(fmt::format("AppendageTemplate can only have 1 loop element. Line number {}",
                                                    loop_elements[1]->GetLineNum()));
            }

            std::vector<const tinyxml2::XMLElement*> commands_elements = getElements("commands");
            if (commands_elements.size() == 1)
            {
                XmlElement commands(commands_elements[0]);

                for (const tinyxml2::XMLElement* ele : commands.getElements("command"))
                {
                    m_commands.emplace_back(ele);
                }
            }
            else if (commands_elements.size() > 1)
            {
                throw ElementException(fmt::format("AppendageTemplate can only have 1 commands_elements element. Line number {}",
                                                    commands_elements[1]->GetLineNum()));
            }

            // If there are any extra attributes, throw an exception
            if (!isAttributesEmpty())
            {
                throw AttributeException(fmt::format("Extra attribute for AppendageTemplate on line number {}",
                                                     xml->GetLineNum()));
            }

            // If there are any extra elements, throw an exception
            if (!isElementsEmpty())
            {
                throw ElementException(fmt::format("Extra element for AppendageTemplate on line number {}",
                                                   xml->GetLineNum()));
            }
        }

        std::shared_ptr<Includes> AppendageTemplate::GetIncludes() const
        {
            return m_includes;
        }

        std::shared_ptr<Constructors> AppendageTemplate::GetConstructors() const
        {
            return m_constructors;
        }

        std::shared_ptr<Setup> AppendageTemplate::GetSetup() const
        {
            return m_setup;
        }

        std::shared_ptr<Loop> AppendageTemplate::GetLoop() const
        {
            return m_loop;
        }

        const std::vector<Command>& AppendageTemplate::GetCommands() const
        {
            return m_commands;
        }
    }
}
