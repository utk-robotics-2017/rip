#include "framework/spine.hpp"

#include <cppfs/fs.h>
#include <cppfs/FileHandle.h>

#include <json.hpp>
#include <appendages/appendage_factory.hpp>
#include <misc/logger.hpp>

namespace rip
{
    namespace framework
    {
        Spine::~Spine()
        {
            stop();
        }

        void Spine::loadDevices(const std::string& arduino_gen_folder, std::vector< std::string >& device_names)
        {
            // device names have not been specified
            if (device_names.empty())
            {
                cppfs::FileHandle arduino_gen_home = cppfs::fs::open(arduino_gen_folder);
                for (const std::string& child : arduino_gen_home.listFiles())
                {
                    cppfs::FileHandle fh = cppfs::fs::open(child);
                    if (fh.isDirectory() && canLoadDevice(arduino_gen_folder, child))
                    {
                        device_names.push_back(child);
                    }
                }
            }
            else
            {
                // Check if all of the specified devices can be loaded
                for (const std::string& device_name : device_names)
                {
                    if (!canLoadDevice(arduino_gen_folder, device_name))
                    {
                        misc::Logger::getInstance()->error(fmt::format("Cannot load {}", device_name));
                        throw CannotLoadDevice(fmt::format("Cannot load {}", device_name));
                    }
                }

            }

            // Create devices
            for (const std::string& device_name : device_names)
            {
                misc::Logger::getInstance()->debug(fmt::format("Loading device {}...", device_name));
                m_devices[device_name] = std::make_shared<cmdmessenger::Device>(fmt::format("/dev/{}", device_name));
                loadConfig(m_devices[device_name], fmt::format("{}/{}/{}_core.json", arduino_gen_folder, device_name, device_name));
            }
        }

        void Spine::stop()
        {
            for (auto iter : m_appendages)
            {
                iter.second->stop();
            }
        }

        bool Spine::diagnostic()
        {
            for (auto iter : m_appendages)
            {
                if (!iter.second->diagnostic())
                {
                    return false;
                }
            }

            return true;
        }

        bool Spine::canLoadDevice(const std::string& arduino_gen_folder, const std::string& device_name) const
        {
            cppfs::FileHandle dev = cppfs::fs::open(fmt::format("/dev/{}", device_name));
            cppfs::FileHandle config = cppfs::fs::open(fmt::format("{}/{}/{}_core.json", arduino_gen_folder, device_name, device_name));
            return dev.exists() && config.exists();
        }

        void Spine::loadConfig(std::shared_ptr<cmdmessenger::Device> device, const std::string& path)
        {
            misc::Logger::getInstance()->debug(fmt::format("Loading appendage config {} ...", path));
            cppfs::FileHandle config_file = cppfs::fs::open(path);
            if (!config_file.exists())
            {
                misc::Logger::getInstance()->error(fmt::format("Cannot find appendage config file {}", path));
                throw FileNotFound(fmt::format("Cannot find {}", path));
            }

            std::unique_ptr<std::istream> in = config_file.createInputStream();

            nlohmann::json config;
            (*in) >> config;

            nlohmann::json commands = config["commands"];

            std::map< std::string, int> command_map;
            for (nlohmann::json::iterator iter = commands.begin(); iter != commands.end(); iter++)
            {
                command_map[iter.key()] = iter.value();
            }

            std::shared_ptr<appendages::AppendageFactory> factory = appendages::AppendageFactory::getInstance();
            nlohmann::json appendages_conf = config["appendages"];
            for (nlohmann::json appendage : appendages_conf)
            {
                if (appendage.find("type") == appendage.end())
                {
                    throw AppendageWithoutType(fmt::format("appendage missing type"));
                }
                std::string appendage_type = appendage["type"];
                if (appendage.find("label") == appendage.end())
                {
                    throw AppendageWithoutLabel(fmt::format("appendage of type {} has no label", appendage_type));
                }
                std::string appendage_label = appendage["label"];
                misc::Logger::getInstance()->debug(fmt::format("Loading appendage {} of type {} ...", appendage_label, appendage_type));
                m_appendages[appendage_label] = factory->makeAppendage(appendage, command_map, device);
            }
        }
    }
} // rip
