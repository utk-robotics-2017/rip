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
#ifndef ROBOT_BASE_HPP
#define ROBOT_BASE_HPP

#include <thread>
#include <map>
#include <string>
#include <chrono>
#include <memory>

#include <cppfs/fs.h>
#include <cppfs/FileHandle.h>

#include <json.hpp>

#include <units/units.hpp>

#include "action.hpp"
#include "spine.hpp"
#include "subsystem.hpp"

namespace rip
{
    namespace framework
    {
        /**
         * Main robot base class
         */
        class RobotBase
        {
        public:
            /**
             * Constructor
             *
             * @param config_path The path to the config file for this robot
             */
            RobotBase(const std::string& config_path);

            /**
             * Destructor
             */
            ~RobotBase();

            /**
             * Initalize robot
             */
            void init();

            /**
             * Builds the routine to run for the robot
             */
            virtual void createRoutine() = 0;

            /**
             * Create the subsystem controllers for the robot
             */
            virtual void createSubsystems(const nlohmann::json& config) = 0;

            /**
             * Start the routine
             */
            void start();

            /**
             * Stop the routine
             */
            void stop();

            /**
             * Run the diagnostics for all of the subsystems and appendages
             */
            void diagnostic();

        protected:
            /**
             * Main loop for the routine
             */
            void run();

        protected:
            bool m_running;
            units::Time m_update_time;
            std::unique_ptr<std::thread> m_thread;
            std::unique_ptr<std::ostream> m_state_file;

            std::vector< std::shared_ptr<Action> > m_routine;

            std::unique_ptr<Spine> m_spine;
            std::map<std::string, std::shared_ptr<Subsystem> > m_subsystems;

            std::string m_config_path;
        };
    }
}

#endif // ROBOT_BASE_HPP
