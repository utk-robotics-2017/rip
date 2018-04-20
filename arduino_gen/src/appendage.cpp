#include "arduino_gen/appendage.hpp"

#include <fmt/format.h>
#include <iostream>
#include <cppfs/fs.h>
#include <cppfs/FileHandle.h>

#include "arduino_gen/exceptions.hpp"

namespace rip
{
    namespace arduinogen
    {
		std::map< std::string, nlohmann::json > Appendage::m_type_cache = std::map< std::string, nlohmann::json >();

        Appendage::Appendage(nlohmann::json j,
                             std::vector<std::string> appendage_data_folders,
                             bool test)
            : m_data(j),
              m_appendage_data_folders(appendage_data_folders)
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

        bool Appendage::has(std::string data_name) const
        {
            return m_data.find(data_name) != m_data.end();
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
                //return fmt::format("\"{}\"", get<std::string>(data_name));
                return get<std::string>(data_name);
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

        void Appendage::testType()
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

                cppfs::FileHandle type_template;
                for (std::string folder : m_appendage_data_folders)
                {
                    type_template = cppfs::fs::open(fmt::format("{}/json/{}.json", folder, type_file));
                    if(type_template.exists() && type_template.isFile())
                    {
                        break;
                    }
                }

                if (!type_template.exists())
                {
                    throw AppendageDataException(fmt::format("Type template file not found for {}", type));
                }

                nlohmann::json j;
                (*type_template.createInputStream()) >> j;
				m_type_cache[type] = j;
            }

			nlohmann::json& type_check = m_type_cache[type];

            // Check if the appendage has all the parameters specified by the tempate
            // and that they are the correct type
			for (nlohmann::json::iterator type_parameter = type_check.begin(); type_parameter != type_check.end(); ++type_parameter)
            {
                if (type_parameter.key() != "core" && m_data.find(type_parameter.key()) == m_data.end())
                {
                    throw AppendageDataException(fmt::format("{} missing on {}", type_parameter.key(), label));
                }

                nlohmann::json parameter = m_data[type_parameter.key()];

				if (type_parameter.key() == "core")
				{
					if (!type_parameter.value().is_array())
					{
						throw AppendageDataException(fmt::format("Core field is not an array for type {}", type));
					}

					nlohmann::json& core_keys = type_parameter.value();
					for (nlohmann::json::iterator core_key = core_keys.begin(); core_key != core_keys.end(); ++core_key)
					{
						if (m_data.find(core_key.value().get<std::string>()) == m_data.end())
						{
							throw AppendageDataException(fmt::format("{} missing core field {}", label, core_key.value().get<std::string>()));
						}

						m_core_fields.emplace_back(core_key.value().get<std::string>());
					}
				}
                else if(type_parameter.value() == "int")
                {
                    if(!parameter.is_number_integer())
                    {
                        throw AppendageDataException(fmt::format("Incorrect type for {} on {}. Should be an integer.",
                                                                 type_parameter.key(), label));
                    }
                }
                else if(type_parameter.value() == "float")
                {
                    if(!parameter.is_number_float())
                    {
                        throw AppendageDataException(fmt::format("Incorrect type for {} on {}. Should be an integer.",
                                                                 type_parameter.key(), label));
                    }
                }
                else if(type_parameter.value() == "string")
                {
                    if(!parameter.is_string())
                    {
                        throw AppendageDataException(fmt::format("Incorrect type for {} on {}. Should be a string.",
                                                                 type_parameter.key(), label));
                    }
                }
                else if(type_parameter.value() == "bool")
                {
                    if(!parameter.is_boolean())
                    {
                        throw AppendageDataException(fmt::format("Incorrect type for {} on {}. Should be a bool.",
                                                                 type_parameter.key(), label));
                    }
                }
                else if(type_parameter.value() == "object")
                {
                    if(!parameter.is_object())
                    {
                        throw AppendageDataException(fmt::format("Incorrect type for {} on {}. Should be an object.",
                                                                 type_parameter.key(), label));
                    }
                }
                else
                {
                    throw AppendageDataException(fmt::format("Unknown Type in template file for {}", type_parameter.key(),
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

        nlohmann::json Appendage::getCoreJson(int index) const
        {
            nlohmann::json json;

            json["type"] = m_data["type"];
            json["label"] = m_data["label"];
            json["index"] = index;

			for (std::string core_key : m_core_fields)
			{
				json[core_key] = m_data[core_key];
			}

            return json;
        }
    }
}
