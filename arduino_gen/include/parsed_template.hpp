#ifndef PARSED_TEMPLATE_H
#define PARSED_TEMPLATE_H

#include <string>
#include <vector>
#include <memory>

#include <json.hpp>

#include "appendage.hpp"

namespace arduinogen
{
    class Constructors;
    class Setup;
    class Loop;
    class Command;

    /**
     * @struct ParsedTemplate
     * @brief Container for the information about a single appendage type
     *        after parsing its respective template file
     */
    struct ParsedTemplate
    {
        std::vector< std::shared_ptr<Appendage> > appendages;

        std::vector< std::string > includes;
        std::unique_ptr<Constructors> constructors;
        std::unique_ptr<Setup> setup;
        std::unique_ptr<Loop> loop;
        std::vector< Command > commands;
        std::string extra;
        nlohmann::json core;
    };
}

#endif // PARSEDTEMPLATE_H
