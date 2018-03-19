#include <iostream>

#include <args.hxx>
#include <fmt/format.h>
#include <cppfs/cppfs.h>
#include <cppfs/fs.h>
#include <cppfs/FileHandle.h>

#include "arduino_gen/arduino_gen.hpp"

#include <misc/exception_base.hpp>

int main(int argc, char* argv[])
{
    args::ArgumentParser parser("ArduinoGen generates arduino code to be used with RIP.");
    args::HelpFlag help(parser, "help", "Display this help menu", {'h', "help"}, args::Options::Single);
    args::CompletionFlag completion(parser, {"complete"});
    parser.Prog("arduino_gen");

    args::ValueFlag<std::string> arduino(parser, "ARDUINO", "Name of the arduino", {'a', "arduino"}, args::Options::Required | args::Options::Single);
    args::ValueFlag<std::string> config(parser, "CONFIG", "Location of the config json file", {'c', "config"}, args::Options::Required | args::Options::Single);
    args::ValueFlag<std::string> parent_folder(parser, "PARENT_FOLDER", "Parent folder of the folder to put all the output files", {"parent_folder"}, "/Robot/CurrentArduinoCode", args::Options::Single);
    args::ValueFlag<std::string> appendages(parser, "APPENDAGES_FOLDER", "Folder of where to look for appendage files", {"appendages"}, "./appendages", args::Options::Single);
    args::Flag noCopy(parser, "NO_COPY", "Don't copy existing files", {"no_copy"}, args::Options::Single);
    args::Flag noGit(parser, "NO_GIT", "Don't git commit the files when uploading", {"no_git"}, args::Options::Single);

    args::Group buildUploadGroup(parser, "Build or Upload arduino code", args::Group::Validators::AtMostOne);
    args::Flag build(buildUploadGroup, "build", "Build the ino file into something that can be uploaded to the Arduino", {'b', "build"}, args::Options::Single);
    args::Flag upload(buildUploadGroup, "upload", "Build the ino file and upload that onto the arduino", {'u', "upload"}, args::Options::Single);

    try
    {
        parser.ParseCLI(argc, argv);
    }
    catch (args::Completion e)
    {
        std::cout << e.what();
        return 0;
    }
    catch (args::Help)
    {
        std::cout << parser;
        return 0;
    }
    catch (args::ParseError e)
    {
        std::cerr << "ParseError: " << e.what() << std::endl << std::endl;
        std::cerr << parser;
        return EXIT_FAILURE;
    }
    catch (args::RequiredError e)
    {
        std::cerr << "RequiredError: " << e.what() << std::endl << std::endl;
        std::cerr << parser << std::endl;
        return EXIT_FAILURE;
    }
    catch (args::ValidationError e)
    {
        std::cerr << "ValidationError: " << e.what() << std::endl << std::endl;
        std::cerr << parser << std::endl;
        return EXIT_FAILURE;
    }

    /*
     * args::get(arduino) is guarenteed to have a value
     * args::get(config) is guarenteed to have a value
     * args::get(parent_folder) is guarenteed to have a value, defaults to: /Robot/CurrentArduinoCode
     * args::get(appendages) is guarenteed to have a value, defaults to: ./appendages
     * args::get(build) defaults to: false
     * args::get(upload) defaults to: false
     * build and upload are guarenteed to be mutually exclusive
    */

    cppfs::FileHandle config_fh = cppfs::fs::open(args::get(config));
    if (!config_fh.exists() || !config_fh.isFile())
    {
        std::cerr << fmt::format("The config file \"{}\" doesn't exist.", args::get(config)) << std::endl;
        return EXIT_FAILURE;
    }

    cppfs::FileHandle parent_folder_fh = cppfs::fs::open(args::get(parent_folder));
    if (!parent_folder_fh.exists() || !parent_folder_fh.isDirectory())
    {
        std::cerr << fmt::format("The parent folder \"{}\" doesn't exist.", args::get(parent_folder)) << std::endl;
        return EXIT_FAILURE;
    }

    cppfs::FileHandle appendages_fh = cppfs::fs::open(args::get(appendages));
    if (!appendages_fh.exists() || !appendages_fh.isDirectory())
    {
        std::cerr << fmt::format("The appendages folder \"{}\" doesn't exist.", args::get(appendages)) << std::endl;
        return EXIT_FAILURE;
    }

    rip::arduinogen::ArduinoGen ag(args::get(arduino), args::get(parent_folder), "/Robot/CurrentArduinoCode", args::get(appendages));

    try
    {
        ag.readConfig(args::get(config));
    }
    catch (rip::utilities::ExceptionBase e)
    {
        std::cerr << "ArduinoGen failed to read the config file.\n" << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    try
    {
        ag.generateOutput(!args::get(noCopy));
    }
    catch (rip::utilities::ExceptionBase e)
    {
        std::cerr << "ArduinoGen failed to generate the output.\n" << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "Arduino code generated." << std::endl;

    // REVIEW: Do we want to check the build/upload flags before generating the new .ino?

    // REVIEW: Is using std::system fine?

    if (build)
    {
        std::system(fmt::format("pio run --project-dir {}/{}", args::get(parent_folder), args::get(arduino)).c_str());
    }
    else if (upload)
    {
        if (noGit)
        {
            std::system(fmt::format("pio run --project-dir {}/{} -t upload", args::get(parent_folder), args::get(arduino)).c_str());
        }
        else if (args::get(parent_folder) != "/Robot/CurrentArduinoCode")
        {
            std::cerr << "ArduinoGen will not upload when the parent_folder is not \"/Robot/CurrentArduinoCode\".\n"
                         "You can override this using --no_git" << std::endl;
            return EXIT_FAILURE;
        }
        else
        {
            std::system(fmt::format("sh {}/{}/upload.sh", args::get(parent_folder), args::get(arduino)).c_str());
        }
    }

    return EXIT_SUCCESS;
}
