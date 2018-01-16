#include "arduino_gen.hpp"

#include <fstream>
#include <sys/stat.h>
#include <set>

#include <tinyxml2.h>
#include <fmt/format.h>
#include <json.hpp>
#include <cppfs/fs.h>
#include <cppfs/FileHandle.h>

#include "appendage_template.hpp"
#include "appendage.hpp"
#include "includes.hpp"
#include "setup.hpp"
#include "constructors.hpp"
#include "argument.hpp"
#include "loop.hpp"
#include "command.hpp"
#include "utils.hpp"
#include "xml_utils.hpp"

namespace rip
{
    namespace arduinogen
    {

        ArduinoGen::ArduinoGen(std::string arduino, std::string parent_folder)
            : m_arduino(arduino)
            , m_parent_folder(parent_folder)
        {
            assert(m_arduino.size() > 0);
            assert(m_parent_folder.size() > 0);

            setupFolder();
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
                                                   m_appendages)));
            }
        }

        void ArduinoGen::generateOutput()
        {
            fillCodeTemplate();

            // Write the Arduino Code
            std::ofstream source;
            source.open(fmt::format("{0}/{1}/src/{1}.ino", m_parent_folder, m_arduino),
                        std::ios::out | std::ios::trunc);
            source << m_code;
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

        void ArduinoGen::fillCodeTemplate()
        {
            loadTemplates();

            std::ifstream code_template;
            code_template.open("code_template.txt");
            m_code = "";
            std::string line = "";
            while(getline(code_template, line))
            {
                m_code += line;
            }
            code_template.close();

            replace(m_code, "<<<includes>>>", getIncludes());
            replace(m_code, "<<<constructors>>>", getConstructors());
            replace(m_code, "<<<setup>>>", getSetup());
            replace(m_code, "<<<loop>>>", getLoop());
            replace(m_code, "<<<commands>>>", getCommandEnums());
            replace(m_code, "<<<command_attaches>>>", getCommandAttaches());
            replace(m_code, "<<<command_callbacks>>>", getCommandCallbacks());
            replace(m_code, "<<<extra>>>", getExtras());
        }

        void ArduinoGen::replace(std::string& base, const std::string& replacee,
                                 const std::string& replacer)
        {
            base.replace(base.find(replacee), replacee.length(), replacer);
        }

        void ArduinoGen::loadTemplates()
        {
            // auto -> std::multimap<std::string, std::shared_ptr<Appendage>>::iterator
            for(auto it = m_appendages.begin(), end = m_appendages.end(); it != end;
                    it = m_appendages.upper_bound(it->first))
            {
                std::string type_file = it->first;
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

                std::vector<std::shared_ptr<Appendage>> appendages = mmap_to_vector(m_appendages, it->first);

                tinyxml2::XMLDocument doc;
                loadXmlFile(doc, fmt::format("appendages/arduino_gen/{0}.template", type_file), { "code", "setup", "loop" });

                m_appendage_templates.emplace_back(doc.FirstChildElement("appendage-template"), appendages);
            }

            // REVIEW: Do we need to sort this?
            /*
            sort(begin(m_appendage_templates), end(m_appendage_templates), [](AppendageTemplate a, AppendageTemplate b) {
                // TODO: Make a key to sort on
            });
            */
        }

        std::string ArduinoGen::getIncludes()
        {
            // Loop through all the includes and let the set handle duplicates
            std::set<std::string> includes;
            for(const AppendageTemplate& at : m_appendage_templates)
            {
                for(std::string& include : at.GetIncludes()->GetIncludes())
                {
                    includes.insert(include);
                }
            }

            std::string rv = "";
            for(std::string include : includes)
            {
                rv += fmt::format("#include {}\n", include);
            }
            return rv;
        }


        std::string ArduinoGen::getConstructors()
        {
            std::string rv = "";
            for(const AppendageTemplate& at : m_appendage_templates)
            {
                if(at.GetConstructors())
                {
                    rv += at.GetConstructors()->toString(at.GetAppendages()) + "\n";
                }
            }
            return rv;
        }

        std::string ArduinoGen::getSetup()
        {
            std::string rv = "";
            for(const AppendageTemplate& at : m_appendage_templates)
            {
                if(at.GetSetup())
                {
                    rv += at.GetSetup()->toString(at.GetAppendages()) + "\n";
                }
            }
            return rv;
        }

        std::string ArduinoGen::getLoop()
        {
            std::string rv = "";
            for(const AppendageTemplate& at : m_appendage_templates)
            {
                if(at.GetLoop())
                {
                    rv += at.GetLoop()->toString(at.GetAppendages()) + "\n";
                }
            }
            return rv;
        }

        std::string ArduinoGen::getCommandEnums()
        {
            std::string rv = "";
            m_commands["kAcknowledge"] = 0;
            m_commands["kError"] = 1;
            m_commands["kUnknown"] = 2;
            m_commands["kSetLed"] = 3;
            m_commands["kPing"] = 4;
            m_commands["kPingResult"] = 5;
            m_commands["kPong"] = 6;
            int cmd_idx = 7;

            std::string cmd_id;
            std::string result_id;
            for(const AppendageTemplate& at : m_appendage_templates)
            {
                for(const std::shared_ptr<Command>& cmd : at.GetCommands())
                {
                    cmd_id = cmd->getId();
                    rv += fmt::format("\t{},\n", cmd_id);
                    m_commands[cmd_id] = cmd_idx;
                    cmd_idx ++;

                    result_id = cmd->getResultId();
                    if(result_id.size())
                    {
                        rv += fmt::format("\t{},\n", result_id);
                        m_commands[result_id] = cmd_idx;
                        cmd_idx ++;
                    }
                }
            }
            rv = rv.substr(0, rv.size() - 2); // Remove last comma

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
            return rv;
        }

        std::string ArduinoGen::getExtras()
        {

            std::string rv = "";
            /*
            for(const AppendageTemplate& at : m_appendage_templates)
            {
                rv += at.extra + "\n";
            }
            */
            return rv;
        }

        std::string ArduinoGen::getCoreConfig()
        {
            // TODO: Implement
            return "";
        }

        std::string ArduinoGen::getUploadScript()
        {
            // TODO: Implement
            return "";
        }
    }
}
