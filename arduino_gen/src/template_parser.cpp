#include "template_parser.hpp"

#include <cstring>

#include <tinyxml2.h>
#include <fmt/format.h>

#include "exceptions.hpp"
#include "appendage.hpp"
#include "constructors.hpp"
#include "argument.hpp"
#include "setup.hpp"
#include "loop.hpp"
#include "command.hpp"

namespace arduinogen
{
    namespace templateparserdetail
    {
        ParsedTemplate parseTemplate(std::string template_filepath,
                                     std::vector< std::shared_ptr<Appendage> > appendages)
        {
            tinyxml2::XMLDocument doc;
            doc.LoadFile(template_filepath.c_str());

            ParsedTemplate pt;

            tinyxml2::XMLElement* includes = doc.FirstChildElement("includes");
            if(includes)
            {
                pt.includes = parseIncludes(includes);
            }

            tinyxml2::XMLElement* constructors = doc.FirstChildElement("constructors");
            if(constructors)
            {
                pt.constructors.reset(new Constructors(constructors));
            }
            else
            {
                pt.constructors.reset(nullptr);
            }

            tinyxml2::XMLElement* setup = doc.FirstChildElement("setup");
            if(setup)
            {
                pt.setup.reset(new Setup(setup));
            }
            else
            {
                pt.setup.reset(nullptr);
            }

            tinyxml2::XMLElement* loop = doc.FirstChildElement("loop");
            if(loop)
            {
                pt.loop.reset(new Loop(loop));
            }
            else
            {
                pt.loop.reset(nullptr);
            }

            tinyxml2::XMLElement* commands = doc.FirstChildElement("commands");
            if(commands)
            {
                pt.commands = parseCommands(commands);
            }

            tinyxml2::XMLElement* extras = doc.FirstChildElement("extras");
            if(extras)
            {
                pt.extra = parseExtras(extras);
            }

            pt.core = parseCoreConfig(doc, appendages);

            pt.appendages = appendages;

            return pt;
        }

        std::vector<std::string> parseIncludes(tinyxml2::XMLElement* includes)
        {
            std::vector< std::string > rv;

            // Loop through individual include's
            for(tinyxml2::XMLElement* include = includes->FirstChildElement("include"); include != nullptr;
                    include = include->NextSiblingElement("include"))
            {
                rv.push_back(include->GetText());
            }
            return rv;
        }

        std::vector<Command> parseCommands(tinyxml2::XMLElement* commands)
        {
            std::vector<Command> rv;

            if(!commands)
            {
                return rv;
            }

            for(tinyxml2::XMLElement* command = commands->FirstChildElement("command"); command != nullptr;
                    command = command->NextSiblingElement("command"))
            {
                rv.emplace_back(command);
            }

            return rv;
        }

        std::string parseExtras(tinyxml2::XMLElement* extras)
        {
            // TODO
            return "";
        }

        nlohmann::json parseCoreConfig(tinyxml2::XMLDocument& core,
                                       std::vector< std::shared_ptr<Appendage> >& appendages)
        {
            // TODO
            return nlohmann::json();
        }
    }
}
