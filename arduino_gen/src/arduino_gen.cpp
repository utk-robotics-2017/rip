#include "arduino_gen/arduino_gen.hpp"

#include <fstream>
#include <sys/stat.h>
#include <set>

#include <tinyxml2.h>
#include <fmt/format.h>
#include <json.hpp>
#include <cppfs/cppfs.h>
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

        ArduinoGen::ArduinoGen(std::string arduino, std::string parent_folder, std::string current_arduino_code_dir, std::string appendage_data_folder)
            : m_arduino(arduino)
            , m_parent_folder(parent_folder)
            , m_current_arduino_code_dir(current_arduino_code_dir)
            , m_appendage_data_folder(appendage_data_folder)
        {
            assert(m_arduino.size() > 0);
            assert(m_parent_folder.size() > 0);
        }

        void ArduinoGen::readConfig(std::string config_path)
        {
            using namespace cppfs;

            FileHandle config_file = fs::open(config_path);

            if (!config_file.exists() || !config_file.isFile())
            {
                throw FileIoException("Config file does not exist");
            }

            config = config_file.readFile();

            // Read the config file in as json
            nlohmann::json config_json = nlohmann::json::parse(config);

            for(nlohmann::json appendage_json : config_json)
            {
                std::string type = appendage_json["type"];

                m_appendages.insert(std::make_pair(type, std::make_shared<Appendage>(appendage_json,
                                                   m_appendages, m_appendage_data_folder, true)));
            }

            loadTemplates();
            loadCommandEnums();
        }

        void ArduinoGen::generateOutput(bool copyOldFiles)
        {
            using namespace cppfs;

            // Read or generate platformio.ini
            std::string platformio_str;
            FileHandle platformio_fh = fs::open(fmt::format("{}/{}/platformio.ini", m_parent_folder, m_arduino));
            if (copyOldFiles && platformio_fh.exists() && platformio_fh.isFile())
            {
                platformio_str = platformio_fh.readFile();
            }
            else
            {
                platformio_str = getPlatformIo();
            }

            // Read or generate serial.sh
            FileHandle serial_fh = fs::open(fmt::format("{0}/{1}/serial.sh", m_parent_folder, m_arduino));
            std::string serial_str;
            if (copyOldFiles && serial_fh.exists() && serial_fh.isFile())
            {
                serial_str = serial_fh.readFile();
            }
            else
            {
                serial_str = getSerialScript();
            }

            // Read or generate upload.sh
            FileHandle upload_fh = fs::open(fmt::format("{0}/{1}/upload.sh", m_parent_folder, m_arduino));
            std::string upload_str;
            if (copyOldFiles && upload_fh.exists() && upload_fh.isFile())
            {
                upload_str = upload_fh.readFile();
            }
            else
            {
                upload_str = getUploadScript();
            }

            // TODO: Move generation code to before the device folder gets removed.
            // If an exception occurs during generation, we don't want to remove the old device_folder.

            // REVIEW: Do we need to change the file and directory owners in addition to setting permissions?

            // Delete and create output dir
            FileHandle device_folder = fs::open(fmt::format("{}/{}", m_parent_folder, m_arduino));
            if (device_folder.exists() && device_folder.isDirectory())
            {
                device_folder.removeDirectoryRec();
            }
			if (!device_folder.createDirectory())
			{
				throw FileIoException(fmt::format("Could not create device folder: \"{}\"", device_folder.path()));
			}
            // device_folder.setPermissions(FilePermissions::UserRead  | FilePermissions::UserWrite  | FilePermissions::UserExec  |
            //                              FilePermissions::GroupRead | FilePermissions::GroupWrite | FilePermissions::GroupExec |
            //                              FilePermissions::OtherRead);// | FilePermissions::OtherWrite | FilePermissions::OtherExec);

            // Create src dir
            FileHandle source_folder = fs::open(fmt::format("{}/{}/{}", m_parent_folder, m_arduino, "src"));
			if (!source_folder.createDirectory())
			{
				throw FileIoException(fmt::format("Could not create source folder: \"{}\"", source_folder.path()));
			}
            // source_folder.setPermissions(FilePermissions::UserRead  | FilePermissions::UserWrite  | FilePermissions::UserExec  |
            //                              FilePermissions::GroupRead | FilePermissions::GroupWrite | FilePermissions::GroupExec |
            //                              FilePermissions::OtherRead);// | FilePermissions::OtherWrite | FilePermissions::OtherExec);

            // Create ino file
            FileHandle source = fs::open(fmt::format("{0}/{1}/src/{1}.ino", m_parent_folder, m_arduino));
			if (!source.writeFile(getArduinoCode()))
			{
				throw FileIoException(fmt::format("Could not create source file: \"{}\"", source.path()));
			}
            source.setPermissions(FilePermissions::UserRead  | FilePermissions::UserWrite  |
                                  FilePermissions::GroupRead | FilePermissions::GroupWrite |
                                  FilePermissions::OtherRead);// | FilePermissions::OtherWrite);

            // Create config file
            FileHandle json_config = fs::open(fmt::format("{0}/{1}/{1}.json", m_parent_folder, m_arduino));
			if (!json_config.writeFile(config))
			{
				throw FileIoException(fmt::format("Could not create json config: \"{}\"", json_config.path()));
			}
            json_config.setPermissions(FilePermissions::UserRead  | FilePermissions::UserWrite  |
                                       FilePermissions::GroupRead | FilePermissions::GroupWrite |
                                       FilePermissions::OtherRead);// | FilePermissions::OtherWrite);

            // Create core file
            FileHandle core_config = fs::open(fmt::format("{0}/{1}/{1}_core.json", m_parent_folder, m_arduino));
            if (!core_config.writeFile(getCoreConfig()))
			{
				throw FileIoException(fmt::format("Could not create core config: \"{}\"", core_config.path()));
			}
            core_config.setPermissions(FilePermissions::UserRead  | FilePermissions::UserWrite  |
                                       FilePermissions::GroupRead | FilePermissions::GroupWrite |
                                       FilePermissions::OtherRead);// | FilePermissions::OtherWrite);

            // Create platformio.ini
            FileHandle platformio_ini = fs::open(fmt::format("{}/{}/platformio.ini", m_parent_folder, m_arduino));
			if (!platformio_ini.writeFile(platformio_str))
			{
				throw FileIoException(fmt::format("Could not create platformio.ini: \"{}\"", platformio_ini.path()));
			}
            platformio_ini.setPermissions(FilePermissions::UserRead  | FilePermissions::UserWrite  |
                                          FilePermissions::GroupRead | FilePermissions::GroupWrite |
                                          FilePermissions::OtherRead);// | FilePermissions::OtherWrite);

            // Create serial script
            FileHandle serial = fs::open(fmt::format("{0}/{1}/serial.sh", m_parent_folder, m_arduino));
			if (!serial.writeFile(serial_str))
			{
				throw FileIoException(fmt::format("Could not create serial.sh: \"{}\"", serial.path()));
			}
            serial.setPermissions(FilePermissions::UserRead  | FilePermissions::UserWrite  | FilePermissions::UserExec  |
                                  FilePermissions::GroupRead | FilePermissions::GroupWrite | FilePermissions::GroupExec |
                                  FilePermissions::OtherRead);// | FilePermissions::OtherWrite | FilePermissions::OtherExec);

            // Create upload script
            FileHandle upload = fs::open(fmt::format("{0}/{1}/upload.sh", m_parent_folder, m_arduino));
			if (!upload.writeFile(upload_str))
			{
				throw FileIoException(fmt::format("Could not create upload.sh: \"{}\"", upload.path()));
			}
            upload.setPermissions(FilePermissions::UserRead  | FilePermissions::UserWrite  | FilePermissions::UserExec  |
                                  FilePermissions::GroupRead | FilePermissions::GroupWrite | FilePermissions::GroupExec |
                                  FilePermissions::OtherRead);// | FilePermissions::OtherWrite | FilePermissions::OtherExec);
        }

        std::string ArduinoGen::getArduinoCode()
        {
			cppfs::FileHandle code_template_fh = cppfs::fs::open("code_template.txt");
			if (!code_template_fh.exists() || !code_template_fh.isFile())
			{
				throw FileIoException("code_template.txt does not exist");
			}

            std::unique_ptr<std::istream> code_template_istream = code_template_fh.createInputStream();
            std::string code_template(std::istreambuf_iterator<char>(*code_template_istream), {});

            return fmt::format(code_template,
                fmt::arg("max_callbacks", m_commands.size()),
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
            int i = 0;
            for (auto it : sorted_commands)
            {
                rv += fmt::format("\t{} = {},\n", it.second, i++);
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
            return fmt::format(
                "#!/usr/bin/env bash\n"
                "\n"
                "cd {current_arduino_code_dir}/{arduino}\n" // REVIEW: Shouldn't this be m_parent_folder
                "\n"
                "git add -A\n"
                "git commit -m \"New code for {arduino}\"\n"
                "git push\n"
                "\n"
                "pio run -t upload\n",
                fmt::arg("current_arduino_code_dir", m_current_arduino_code_dir),
                fmt::arg("arduino", m_arduino)
            );
        }

        std::string ArduinoGen::getSerialScript()
        {
            return fmt::format(
                "#!/usr/bin/env bash\n"
                "\n"
                "picocom /dev/{} -b 115200 --echo\n",
                fmt::arg("arduino", m_arduino)
            );
        }

        std::string ArduinoGen::getPlatformIo()
        {
            return fmt::format(
                "[platformio]\n"
                "lib_dir = /Robot/ArduinoLibraries\n" // REVIEW: Should this be hard coded?
                "env_default = {0}\n"
                "\n"
                "[env:{0}]\n"
                "platform = atmelavr\n"
                "framework = arduino\n"
                "board = uno\n"
                "upload_port = /dev/{0}\n",
                m_arduino
            );
        }
    }
}
