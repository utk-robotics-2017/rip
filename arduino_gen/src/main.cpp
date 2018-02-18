#include <iostream>

#include <args.hxx>
#include <fmt/format.h>

#include "arduino_gen/arduino_gen.hpp"

int main(int argc, char* argv[])
{
    /*
    if (argc != 2)
    {
        std::cerr << "usage: ./arduino_gen <config_file>" << std::endl;
        return EXIT_FAILURE;
    }

    std::unique_ptr<rip::arduinogen::ArduinoGen> ag = std::unique_ptr<rip::arduinogen::ArduinoGen>(new rip::arduinogen::ArduinoGen("mega", ".", "/Robot/CurrentArduinoCode", "test/data/arduino_gen"));

    ag->readConfig(argv[1], false);

    std::cout << ag->getArduinoCode() << std::endl;
    */

    args::ArgumentParser parser("ArduinoGen generates arduino code to be used with RIP.");
    args::HelpFlag help(parser, "help", "Display this help menu", {'h', "help"}, args::Options::Single);
    args::CompletionFlag completion(parser, {"complete"});
    parser.Prog("arduino_gen");

    args::ValueFlag<std::string> arduino(parser, "ARDUINO", "Name of the arduino", {'a', "arduino"}, args::Options::Required | args::Options::Single);
    args::ValueFlag<std::string> config(parser, "CONFIG", "Location of the config json file", {'c', "config"}, args::Options::Required | args::Options::Single);
    args::ValueFlag<std::string> parent_folder(parser, "PARENT_FOLDER", "Parent folder of the folder to put all the output files", {"parent_folder"}, "/Robot/CurrentArduinoCode", args::Options::Single);
    args::ValueFlag<std::string> appendages(parser, "APPENDAGES_FOLDER", "Folder of where to look for appendage files", {"appendages"}, "./appendages", args::Options::Single);
    args::Flag noCopy(parser, "NO_COPY", "Don't copy existing files", {'n', "no_copy"}, args::Options::Single);

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

    rip::arduinogen::ArduinoGen ag(args::get(arduino), args::get(parent_folder), "/Robot/CurrentArduinoCode", args::get(appendages));

    ag.readConfig(args::get(config));
    ag.generateOutput(!args::get(noCopy));

    // TODO: Build and Upload

    return EXIT_SUCCESS;
}
