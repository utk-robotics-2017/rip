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
#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <vector>
#include <memory>
#include <time.h>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/ansicolor_sink.h>
#include <cppfs/FileHandle.h>
#include <cppfs/fs.h>
#include <misc/constants.hpp>

namespace rip
{
    namespace misc
    {
        class Logger
        {
        public:
            static std::shared_ptr<spdlog::logger> getInstance()
            {
                static bool first = true;
                if (first)
                {
                    std::vector<spdlog::sink_ptr> sinks;
                    sinks.push_back(std::make_shared<spdlog::sinks::ansicolor_stdout_sink_mt>());

                    cppfs::FileHandle logs_folder = cppfs::fs::open("logs");
                    if(!logs_folder.exists())
                    {
                        logs_folder.createDirectory();
                    }

                    time_t now;
                    time(&now);
                    char curtime_iso[sizeof "2011-10-08T07:07:09Z"];
                    strftime(curtime_iso, sizeof curtime_iso, "%FT%TZ", gmtime(&now));
                    // this will work too, if your compiler doesn't support %F or %T:
                    //strftime(buf, sizeof buf, "%Y-%m-%dT%H:%M:%SZ", gmtime(&now));

                    sinks.push_back(std::make_shared<spdlog::sinks::simple_file_sink_mt>(fmt::format("logs/{}.txt", curtime_iso)));
                    std::shared_ptr<spdlog::logger> logger = std::make_shared<spdlog::logger>(constants::kLoggerName, begin(sinks),
                            end(sinks));
                    spdlog::set_pattern("[%H:%M:%S %z] [thread %t] [%I] %v");
                    spdlog::register_logger(logger);
                    spdlog::set_level(spdlog::level::warn);

                    first = false;
                }

                return spdlog::get(constants::kLoggerName);
            }
        };
    }
}

#endif // LOGGER_HPP
