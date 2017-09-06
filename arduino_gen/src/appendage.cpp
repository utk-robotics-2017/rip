#include "appendage.hpp"

#include <fmt/format.h>
#include <path.hpp>
#include <iostream>

#include "exceptions.hpp"

namespace rip
{
    namespace arduinogen
    {
        std::map< std::string, std::map< std::string, std::string> > Appendage::m_type_cache =
            std::map< std::string, std::map< std::string, std::string> >();

        Appendage::Appendage(nlohmann::json j,
                             std::multimap< std::string, std::shared_ptr<Appendage> >& appendages, bool test)
            : m_data(j)
        {
            if(test)
            {
                testType();
            }
        }

        std::string Appendage::getLabel() const
        {
            if (m_data.find("label") == m_data.end())
            {
                throw AppendageDataException("label not found");
            }
            return get<std::string>("label");
        }

        std::string Appendage::getType() const
        {
            if (m_data.find("type") == m_data.end())
            {
                throw AppendageDataException("type not found");
            }
            return get<std::string>("type");
        }

        std::string Appendage::getString(std::string data_name) const
        {

            if (m_data.find(data_name) == m_data.end())
            {
                throw AppendageDataException(fmt::format("{} not found", data_name));
            }
            else if(m_data[data_name].is_number_float())
            {
                return std::to_string(get<float>(data_name));
            }
            else if(m_data[data_name].is_number_integer())
            {
                return std::to_string(get<int>(data_name));
            }
            else if(m_data[data_name].is_boolean())
            {
                return get<bool>(data_name) ? "true" : "false";
            }
            else if(m_data[data_name].is_string())
            {
                return fmt::format("\"{}\"", get<std::string>(data_name));
            }
            else
            {
                throw AppendageDataException(fmt::format("{} of unknown type", data_name));
            }
        }

        bool Appendage::isType(std::string data_name, std::string type) const
        {
            if(m_data.find(data_name) == m_data.end())
            {
                throw AppendageDataException(fmt::format("{} not found", data_name));
            }

            if(type == "int")
            {
                return m_data[data_name].is_number_integer();
            }
            else if(type == "float")
            {
                return m_data[data_name].is_number_float();
            }
            else if(type == "string")
            {
                return m_data[data_name].is_string();
            }
            else if(type == "bool")
            {
                return m_data[data_name].is_boolean();
            }
            else
            {
                throw AppendageDataException(fmt::format("unknown type {}", type));
            }
        }

        void Appendage::testType() const
        {

            std::string type = getType();
            if(!type.size())
            {
                throw AppendageDataException("Empty Type");
            }

            std::string label = getLabel();
            if(!label.size())
            {
                throw AppendageDataException("Empty label");
            }

            if(m_type_cache.find(type) == m_type_cache.end())
            {
                std::string type_file = type;
                for(char& c : type_file)
                {
                    if(c == ' ')
                    {
                        c = '_';
                    }
                    else if(c >= 'A' && c <= 'Z')
                    {
                        c += 'a' - 'A';
                    }
                }

                utilities::pathman::Path type_template(fmt::format("appendages/json/{}.json", type_file));
                if(!type_template.exists() || !type_template.isFile())
                {
                    throw AppendageDataException(fmt::format("Type template file not found for {}", type));
                }

                std::unique_ptr<std::ifstream> i = type_template.openInput();

                nlohmann::json j;
                (*i) >> j;
                std::map< std::string, std::string > temp;
                for (nlohmann::json::iterator it = j.begin(); it != j.end(); ++it) {
                    temp[it.key()] = it.value();
                }
                m_type_cache[type] = temp;
            }

            std::map< std::string, std::string >& type_check = m_type_cache[type];

            // Check if the appendage has all the parameters specified by the tempate
            // and that they are the correct type
            for(std::pair< std::string, std::string> type_parameter : type_check)
            {
                if(m_data.find(type_parameter.first) == m_data.end())
                {
                    throw AppendageDataException(fmt::format("{} missing on {}", type_parameter.first, label));
                }

                nlohmann::json parameter = m_data[type_parameter.first];

                if(type_parameter.second == "int")
                {
                    if(!parameter.is_number_integer())
                    {
                        throw AppendageDataException(fmt::format("Incorrect type for {} on {}. Should be an integer.",
                                                                 type_parameter.first, label));
                    }
                }
                else if(type_parameter.second == "float")
                {
                    if(!parameter.is_number_float())
                    {
                        throw AppendageDataException(fmt::format("Incorrect type for {} on {}. Should be an integer.",
                                                                 type_parameter.first, label));
                    }
                }
                else if(type_parameter.second == "string")
                {
                    if(!parameter.is_string())
                    {
                        throw AppendageDataException(fmt::format("Incorrect type for {} on {}. Should be a string.",
                                                                 type_parameter.first, label));
                    }
                }
                else if(type_parameter.second == "bool")
                {
                    if(!parameter.is_boolean())
                    {
                        throw AppendageDataException(fmt::format("Incorrect type for {} on {}. Should be a bool.",
                                                                 type_parameter.first, label));
                    }
                }
                else if(type_parameter.second == "object")
                {
                    if(!parameter.is_object())
                    {
                        throw AppendageDataException(fmt::format("Incorrect type for {} on {}. Should be an object.",
                                                                 type_parameter.first, label));
                    }
                }
                else
                {
                    throw AppendageDataException(fmt::format("Unknown Type in template file for {}", type_parameter.first,
                                                             label));
                }
            }

            for(nlohmann::json::const_iterator it = m_data.begin(); it != m_data.end(); ++it)
            {
                std::string key = it.key();
                if(key != "label" && key != "type" && type_check.find(key) == type_check.end())
                {
                    throw AppendageDataException(fmt::format("Extra parameter {} on {}", key, label));
                }
            }
        }
    }
}
