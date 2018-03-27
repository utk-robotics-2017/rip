/*
 * The RIP License (Revision 0.3):
 * This software is available without warranty and without support.
 * Use at your own risk. Literally. It might delete your filesystem or
 * eat your cat. As long as you retain this notice, you can do whatever
 * you want with this. If we meet some day, you owe me a beer.
 *
 * Go Vols!
 *
 *  __    __  ________  __    __        _______   ______  _______
 * |  \  |  \|        \|  \  /  \      |       \ |      \|       \
 * | $$  | $$ \$$$$$$$$| $$ /  $$      | $$$$$$$\ \$$$$$$| $$$$$$$\
 * | $$  | $$   | $$   | $$/  $$       | $$__| $$  | $$  | $$__/ $$
 * | $$  | $$   | $$   | $$  $$        | $$    $$  | $$  | $$    $$
 * | $$  | $$   | $$   | $$$$$\        | $$$$$$$\  | $$  | $$$$$$$
 * | $$__/ $$   | $$   | $$ \$$\       | $$  | $$ _| $$_ | $$
 *  \$$    $$   | $$   | $$  \$$\      | $$  | $$|   $$ \| $$
 *   \$$$$$$     \$$    \$$   \$$       \$$   \$$ \$$$$$$ \$$
 */
#ifndef SPINE_HPP
#define SPINE_HPP

#include <vector>
#include <memory>
#include <map>

#include <fmt/format.h>

#include <appendages/appendage.hpp>
#include <cmd_messenger/device.hpp>

#include "framework/exceptions.hpp"

namespace rip
{

  namespace diag
  {
      class Diag;
  }

    namespace framework
    {
        /**
         * @class Spine
         * @brief Handles creating all the appendages from the config files and setting up communication
         */
        class Spine
        {
        public:
            Spine() = default;

            ~Spine();

            /**
             * @brief Attempts to connect to all the devices specified. Defaults to attempting all
             *        if none are provided.
             *
             * @param device_names A list of devices to attempt to connect to
             *
             * @exception DeviceNotFound Thrown if a name is specified and the device is not found
             *            in dev
             * @exception DeviceFolderNotFound Thrown if a name is specified and a folder with the
             *            configuration cannot be found
             */
            void loadDevices(const std::string& arduino_gen_folder, std::vector<std::string>& device_names);

            /**
             * True if appendage is present
             * @param  appendage_name name of appendage to search for
             * @returns boolean
             */
            bool has(const std::string& appendage_name) {
              if (m_appendages.find(appendage_name) == m_appendages.end())
              {
                return false;
              }
              return true;
            }

            /**
             * @brief Returns the specified appendage
             *
             * @tparam T The Appendage class type
             *
             * @param appendage_name The name of the appendage to return
             *
             * @returns The specified appendage
             */
            std::shared_ptr<appendages::Appendage> get(const std::string& appendage_name)
            {
                if (m_appendages.find(appendage_name) == m_appendages.end())
                {
                    throw AppendageNotFound(fmt::format("Cannot find appendage named {}", appendage_name));
                }

                return m_appendages[appendage_name];
            }

            /**
             * Stops all the appendages
             */
            void stop();

            /**
             *  Runs a diagnostic for all of the appendages
             */
            bool diagnostic();

        private:
            /**
             * @brief Loads the config file at a specific path
             *
             * @param path The location of the config file
             *
             * @exception FileNotFound Thrown if the config file cannot be found
             * @exception IncorrectConfig Thrown if the config has incorrect information in it
             */
            void loadConfig(std::shared_ptr<cmdmessenger::Device> device, const std::string& path);

            /**
             * @brief Checks if possible to load a device
             *
             * @param device_name The name of the device
             *
             * @returns Whether the device can be found in dev and if it has a folder with a config file in it
             */
            bool canLoadDevice(const std::string& arduino_gen_folder, const std::string& device_name) const;

            std::map< std::string, std::shared_ptr<cmdmessenger::Device> > m_devices;
            std::map< std::string, std::shared_ptr<appendages::Appendage> > m_appendages;

            friend class diag::Diag;
        }; // class Spine
    }
} // namespace rip

#endif // SPINE_HPP
