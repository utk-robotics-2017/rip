#include "arduino_gen/arduino_gen.hpp"

#include <fstream>
#include <sys/stat.h>
#include <set>

#include <tinyxml2.h>
#include <fmt/format.h>
#include <json.hpp>
#include <cppfs/fs.h>
#include <cppfs/FileHandle.h>

#include "arduino_gen/appendage_template.hpp"
#include "arduino_gen/appendage.hpp"
#include "arduino_gen/includes.hpp"
#include "arduino_gen/setup.hpp"
#include "arduino_gen/constructors.hpp"
#include "arduino_gen/argument.hpp"
#include "arduino_gen/loop.hpp"
#include "arduino_gen/command.hpp"
#include "arduino_gen/utils.hpp"
#include "arduino_gen/xml_utils.hpp"
#include "arduino_gen/exceptions.hpp"

namespace rip
{
    namespace arduinogen
    {

        ArduinoGen::ArduinoGen(std::string arduino, std::string parent_folder, std::string appendage_data_folder, bool testing)
            : m_arduino(arduino)
            , m_parent_folder(parent_folder)
            , m_appendage_data_folder(appendage_data_folder)
        {
            assert(m_arduino.size() > 0);
            assert(m_parent_folder.size() > 0);

            if (!testing)
            {
                setupFolder();
            }
        }

        void ArduinoGen::setupFolder()
        {
            cppfs::FileHandle device_folder = cppfs::fs::open(fmt::format("{}/{}", m_parent_folder, m_arduino));

            if (device_folder.exists() && device_folder.isDirectory())
            {
                device_folder.removeDirectory();
            }
            device_folder.createDirectory();
            // TODO: chmod

            cppfs::FileHandle source_folder = cppfs::fs::open(fmt::format("{}/{}/{}", m_parent_folder, m_arduino, "src"));
            source_folder.createDirectory();
            // TODO: chmod
        }

        void ArduinoGen::readConfig(std::string config_path, bool copy)
        {
            if(copy)
            {
                // TODO: copy and chmod
            }

            // Read the config file in as json
            // REVIEW: We could change this to use cppfs
            std::ifstream config_stream;
            config_stream.open(config_path);
            std::string config_string = "";
            std::string line = "";
            while(getline(config_stream, line))
            {
                config_string += line;
            }
            config_stream.close();
            nlohmann::json config = nlohmann::json::parse(config_string);

            for(nlohmann::json appendage_json : config)
            {
                std::string type = appendage_json["type"];

                m_appendages.insert(std::make_pair(type, std::make_shared<Appendage>(appendage_json,
                                                   m_appendages, m_appendage_data_folder, true)));
            }

            loadTemplates();
            loadCommandEnums();
        }

        void ArduinoGen::generateOutput()
        {
            // Write the Arduino Code
            std::ofstream source;
            source.open(fmt::format("{0}/{1}/src/{1}.ino", m_parent_folder, m_arduino),
                        std::ios::out | std::ios::trunc);
            source << getArduinoCode();
            source.close();
            // TODO: chmod

            // Write config for core
            std::ofstream core_config;
            core_config.open(fmt::format("{0}/{1}/core_config.json", m_parent_folder, m_arduino),
                             std::ios::out | std::ios::trunc);
            core_config << getCoreConfig();
            core_config.close();

            // Write build and upload script
            std::ofstream upload;
            upload.open(fmt::format("{0}/{1}/upload.sh", m_parent_folder, m_arduino),
                        std::ios::out | std::ios::trunc);
            upload << getUploadScript();
            upload.close();
        }

        std::string ArduinoGen::getArduinoCode()
        {
            std::unique_ptr<std::istream> code_template_istream = cppfs::fs::open("code_template.txt").createInputStream();
            std::string code_template(std::istreambuf_iterator<char>(*code_template_istream), {});

            return fmt::format(code_template,
                fmt::arg("includes", getIncludes()),
                fmt::arg("constructors", getConstructors()),
                fmt::arg("setup", getSetup()),
                fmt::arg("loop", getLoop()),
                fmt::arg("command_enums", getCommandEnums()),
                fmt::arg("command_attaches", getCommandAttaches()),
                fmt::arg("command_callbacks", getCommandCallbacks())
            );
        }

        void ArduinoGen::loadTemplates()
        {
            for (std::string type : get_mmap_keys(m_appendages))
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

                std::vector<std::shared_ptr<Appendage>> appendages = get_mmap_values_at_index(m_appendages, type);

                tinyxml2::XMLDocument doc;
                loadXmlFile(doc, fmt::format("{0}/xml/{1}.xml", m_appendage_data_folder, type_file), { "code", "setup", "loop" });

                const tinyxml2::XMLElement* element = doc.FirstChildElement("appendage-template");

                if (element == nullptr)
                {
                    throw AppendageDataException("AppendageTemplate is malformed. "
                        "AppendageTemplate xml files must have a root of appendage-template.");
                }

                m_appendage_templates.emplace_back(element, type, appendages);
            }

            sort(begin(m_appendage_templates), end(m_appendage_templates), [](AppendageTemplate a, AppendageTemplate b) {
                return a.GetType() < b.GetType();
            });
        }

        void ArduinoGen::loadCommandEnums()
        {
            int cmd_idx = 0;

            m_commands["kAcknowledge"] = cmd_idx++;
            m_commands["kError"] = cmd_idx++;
            m_commands["kUnknown"] = cmd_idx++;
            m_commands["kSetLed"] = cmd_idx++;
            m_commands["kPing"] = cmd_idx++;
            m_commands["kPingResult"] = cmd_idx++;
            m_commands["kPong"] = cmd_idx++;

            for(const AppendageTemplate& at : m_appendage_templates)
            {
                for(const std::shared_ptr<Command>& cmd : at.GetCommands())
                {
                    m_commands[cmd->getId()] = cmd_idx++;

                    std::string result_id = cmd->getResultId();
                    if(result_id.size())
                    {
                        m_commands[result_id] = cmd_idx++;
                    }
                }
            }
        }

        std::string ArduinoGen::getIncludes()
        {
            // Loop through all the includes and let the set handle duplicates
            std::set<std::string> includes;
            for(const AppendageTemplate& at : m_appendage_templates)
            {
                std::shared_ptr<Includes> appendageIncludes = at.GetIncludes();
                if (appendageIncludes != nullptr)
                {
                    for(std::string& include : appendageIncludes->GetIncludes())
                    {
                        includes.insert(include);
                    }
                }
            }

            std::string rv = "";
            for(std::string include : includes)
            {
                rv += fmt::format("#include {}\n", include);
            }

            if (rv.size() > 0)
            {
                rv.pop_back();
            }

            return rv;
        }


        std::string ArduinoGen::getConstructors()
        {
            std::string rv = "";

            for(const AppendageTemplate& at : m_appendage_templates)
            {
                std::shared_ptr<Constructors> constructors = at.GetConstructors();
                if(constructors != nullptr)
                {
                    rv += constructors->toString(at.GetAppendages()) + "\n";
                }
            }

            if (rv.size() > 0)
            {
                rv.pop_back();
                rv.pop_back();
            }

            return rv;
        }

        std::string ArduinoGen::getSetup()
        {
            std::string rv = "";
            for(const AppendageTemplate& at : m_appendage_templates)
            {
                std::shared_ptr<Setup> setup = at.GetSetup();
                if(setup != nullptr)
                {
                    rv += setup->toString(at.GetAppendages()) + "\n";
                }
            }

            if (rv.size() > 0)
            {
                rv.pop_back();
                rv.pop_back();
            }

            return rv;
        }

        std::string ArduinoGen::getLoop()
        {
            std::string rv = "";
            for(const AppendageTemplate& at : m_appendage_templates)
            {
                std::shared_ptr<Loop> loop = at.GetLoop();
                if(loop)
                {
                    rv += loop->toString(at.GetAppendages()) + "\n";
                }
            }

            if (rv.size() > 0)
            {
                rv.pop_back();
                rv.pop_back();
            }

            return rv;
        }

        std::string ArduinoGen::getCommandEnums()
        {
            std::map<int, std::string> sorted_commands;

            // auto -> std::map<std::string, int>::iterator
            for (auto it : m_commands)
            {
                sorted_commands.emplace(it.second, it.first);
            }

            std::string rv = "";

            for (auto it : sorted_commands)
            {
                rv += fmt::format("\t{},\n", it.second);
            }

            rv.pop_back(); // Remove last newline
            rv.pop_back(); // Remove last comma

            return rv;
        }

        std::string ArduinoGen::getCommandAttaches()
        {
            std::string rv = "";
            for(const AppendageTemplate& at : m_appendage_templates)
            {
                for(const std::shared_ptr<Command>& cmd : at.GetCommands())
                {
                    rv += fmt::format("\tcmdMessenger.attach({}, {});\n", cmd->getId(), cmd->getName());
                }
            }

            if (rv.size() > 0)
            {
                rv.pop_back();
            }

            return rv;
        }

        std::string ArduinoGen::getCommandCallbacks()
        {
            std::string rv = "";
            for(const AppendageTemplate& at : m_appendage_templates)
            {
                for(const std::shared_ptr<Command>& command : at.GetCommands())
                {
                    rv += command->callback(at.GetAppendages().size()) + "\n";
                }
            }

            if (rv.size() > 0)
            {
                rv.pop_back();
                rv.pop_back();
            }

            return rv;
        }

        std::string ArduinoGen::getCoreConfig()
        {
            nlohmann::json config;

            config["commands"] = m_commands;

            config["appendages"] = nlohmann::json::array();
            for(std::string type : get_mmap_keys(m_appendages))
            {
                std::vector<std::shared_ptr<Appendage>> appendages_of_type = get_mmap_values_at_index(m_appendages, type);

                for (int i = 0; i < appendages_of_type.size(); i++)
                {
                    config["appendages"].emplace_back(appendages_of_type[i]->getCoreJson(i));
                }
            }

            return config.dump(4);
        }

        std::string ArduinoGen::getUploadScript()
        {
            // TODO: Implement
            return "";
        }
    }
}
