#ifndef XMLUTILS_HPP
#define XMLUTILS_HPP

#include <string>
#include <vector>

/*
namespace tinyxml2
{
    class XMLDocument;
    class XMLError;
}
*/
#include <tinyxml2.h>

namespace rip
{
    namespace arduinogen
    {
        /**
         * @brief Loads the file, does some preprocessing, then passes the file contents to tinyxml
         *
         * @param filename Name of the file to be loaded
         * @param elements_to_escape Elements whose contents needs to be escaped
         */
        tinyxml2::XMLError loadXmlFile(tinyxml2::XMLDocument& doc, const std::string& filename, const std::vector<std::string>& elements_to_escape);

        /**
         * @brief Escapes xml characters in the string
         *
         * @param String to be encoded
         * @note Currently escapes the following characters `"'<>&`
         */
        std::string xmlEncode(const std::string& str);
    }
}

#endif // XMLUTILS_HPP
