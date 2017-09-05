#include "spine.hpp"

namespace rip
{
    namespace communication
    {
        std::shared_ptr<Spine> Spine::m_singleton = std::shared_ptr<Spine>(nullptr);

        std::shared_ptr<Spine> Spine::getInstance()
        {
            if (!m_singleton)
            {
                m_singleton = std::make_shared<Spine>();
            }

            return m_singleton;
        }

        void Spine::loadDevices(const std::vector< std::string >& device_names)
        {

            // device names have not been specified
            if (device_names.empty())
            {
                pathman::Path arduino_gen_home(Constants::get<std::string>("arduino_gen_home"));
                std::vector<pathman::Path> possible_device_names = arduino_gen_home.getChildDirs();
                for (const std::string& device_name : possible_device_names)
                {
                    if (canLoadDevice(device_name))
                    {
                        device_names.push_back(device_name);
                    }
                }
            }
            else
            {
                // Check if all of the specified devices can be loaded
                for (const std::string& device_name : device_names)
                {
                    if (!canLoadDevice(device_name))
                    {
                        throw CannotLoadDevice(fmt::format("Cannot load {}", device_name));
                    }
                }

            }

            // Create devices
            for (const std::string& device_name : device_names)
            {
                m_devices[device_name] = std::make_shared<cmdmessenger::Device>(fmt::format("/dev/{}", device_name));
                loadConfig(fmt::format("{}/{}/core.json", Constants::get<std::string>("arduino_gen_home"), device_name));
            }
        }

        bool Spine::canLoadDevice(const std::string& device_name) const
        {
            pathman::Path dev(fmt::format("/dev/{}", device_name));
            pathman::Path config(fmt::format("{}/{}/core.json", Constants::get<std::string>("arduino_gen_home"), device_name));
            return dev.exists() && config.exists();
        }

        void loadConfig(std::shared_ptr<cmdmessenger::Device> device)
        {
            if (!config_path.exists())
            {
                throw FileNotFound(fmt::format("Cannot find {}", config_path.str()));
            }

            std::unique_ptr<std::ifstream> in;

            nlohmann::json config;
            in >> config;

            nlohmann::json commands = config["commands"];

            std::map< std::string, int> command_map;
            for (nlohmann::json::iterator iter = commands.begin(); iter != commands.end(); iter++)
            {
                command_map[iter.key()] = iter.value();
            }

            nlohmann::json appendages;
            for (nlohmann::json appendage : appendages)
            {
                if (appendage.find("type") == appendage.end())
                {
                    throw AppendageWithoutType(fmt::format("{} appendage {} missing type", device->getName(), appendage.dump()));
                }

                m_appendages[appendage["type"]] = makeAppendage(appendage, command_map, device);
            }
        }
    }
} // rip
