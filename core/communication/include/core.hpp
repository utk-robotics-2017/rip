#ifndef SPINE_HPP
#define SPINE_HPP

namespace rip
{
    /**
     * @class Spine
     * @brief Handles creaing all the appendages from the config files and setting up communication
     */
    class Spine
    {
        public:
            /**
             * @brief Returns the singleton
             *
             * @returns The singleton
             */
            static std::shared_ptr<Spine> getInstance();

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
            void loadDevices(std::vector<std::string> device_names = {});

            /**
             * @brief Returns the specified appendage
             *
             * @tparam T The Appendage class type
             *
             * @param appendage_name The name of the appendage to return
             *
             * @returns The specified appendage
             */
            template<class T>
            std::shared_ptr<T> get(std::string appendage_name)
            {
                if(m_appendages.find(appendage_name) == m_appendages.end())
                {
                    throw AppendageNotFound(fmt::format("Cannot find appendage named {}", appendage_name));
                }

                return std::dynamic_pointer_cast<T>(m_appendages[appendage_name]);
            }

        private:
            /**
             * @brief Loads the config file at a specific path
             *
             * @param path The location of the config file
             *
             * @exception FileNotFound Thrown if the config file cannot be found
             * @exception IncorrectConfig Thrown if the config has incorrect information in it
             */
            void loadConfig(pathman::Path path);

            /**
             * @brief Checks if possible to load a device
             *
             * @param device_name The name of the device
             *
             * @returns Whether the device can be found in dev and if it has a folder with a config file in it
             */
            bool canLoadDevice(std::string device_name);

            static std::shared_ptr<Spine> m_singleton;

            std::map< std::string, std::shared_ptr<cmdmessenger::Device> > m_devices;
            std::map< std::string, std::shared_ptr<Appendage> > m_appendages;

    }; // class Spine
} // namespace rip

#endif // SPINE_HPP
