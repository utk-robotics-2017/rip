#ifndef TEMPLATEPARSER_HPP
#define TEMPLATEPARSER_HPP

/**
 * @filename templateparser.hpp
 */

#include <string>
#include <vector>
#include <memory>

#include "parsed_template.hpp"

namespace tinyxml2
{
    class XMLDocument;
    class XMLElement;
}

namespace rip
{
    namespace arduinogen
    {
        class Appendage;

        namespace templateparserdetail
        {
            /**
             * @brief Parses a template file and fills a ParsedTemplate object based on the \p appendages
             *
             * @param template_filpath The filepath to the template file
             * @param appendages A list of appendages of the same type
             *
             * @returns A parsed template filled based on the template file at \p template_filepath
             *          and with the information in the Appendage objects in \p appendages
             */
            ParsedTemplate parseTemplate(std::string template_filepath,
                                         std::vector< std::shared_ptr<Appendage> > appendages);

            /**
             * @brief Parses the "includes" section of the template
             *
             * @param includes The "includes" section of the template to parse
             *
             * @returns A list of individual includes
             *
             * @note: Each string only contains the file name of the include (e.g. Something.h)
             *        not the full: #include "Something.h"
             */
            std::vector<std::string> parseIncludes(tinyxml2::XMLElement* includes);

            /**
             * @brief Parses the "commands" section of the template
             *
             * @param commands The "commands" section of the template to parse
             * @param num_appendages The number of appendages of this type
             *
             * @returns A list of Command objects filled with the enum id, callback function name, and callback function
             */
            std::vector<Command> parseCommands(tinyxml2::XMLElement* commands);

            /**
             * @brief Parses the "extras" section of the template
             *
             * @param extras The extras section of the temlate
             * @param appendages A list of appendages of a specific type
             *
             * @returns The code for the extra functions
             */
            std::string parseExtras(tinyxml2::XMLElement* extras);

            /**
             * @brief Parses the "core config" section of the template
             *
             * @param core The core section of the template
             * @param appendages A list of appendages of a specific type
             *
             * @returns The json of the config to be used by core
             */
            nlohmann::json parseCoreConfig(tinyxml2::XMLDocument& core,
                                           std::vector< std::shared_ptr<Appendage> >& appendage);
        }

        using templateparserdetail::parseTemplate;
    }
}

#endif // TEMPLATEPARSER_H
