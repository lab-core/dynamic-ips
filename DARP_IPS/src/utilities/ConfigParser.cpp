//
// Created by Elahe Amiri on 2025-06-23.
//

#include "ConfigParser.h"
#include <iostream>
#include <map>
#include <stdexcept>
#include <algorithm>

// ProgramConfig implementation
ProgramConfig::ProgramConfig() : numVehicles_(0), mainAlgo_(-1), solMode_(-1), saveScratch_(0), initialState_(-1) {}

void ProgramConfig::printConfig() const {
    std::cout << "Configuration:\n";
    std::cout << "  Vehicle folder: " << vehicleFolder_ << "\n";
    std::cout << "  Instance folder: " << instFolder_ << "\n";
    std::cout << "  Number of vehicles: " << numVehicles_ << "\n";
    std::cout << "  Main algorithm: " << mainAlgo_ << "\n";
    std::cout << "  Solution mode: " << solMode_ << "\n";
    std::cout << "  Parameter file: " << paramFile_ << "\n";
    std::cout << "  Scenario name: " << scenario_ << "\n";
    std::cout << "  Save scratch: " << saveScratch_ << "\n";
    std::cout << "  Vehicle file: " << vehicleFileName_ << "\n";

    if (!instanceName_.empty()) {
        std::cout << "  Instance name: " << instanceName_ << "\n";
    } else {
        std::cout << "  Instance source: Read from file\n";
    }
    std::cout << "  Initial state: " << initialState_ << "\n";
}

//-----------------------------------------------------------------------------
//  Configuration parser class
//-----------------------------------------------------------------------------

// ConfigParser implementation
void ConfigParser::printUsage(const char* programName) {
    std::cout << "Usage: " << programName << " <options>\n\n"
              << "Required arguments:\n"
              << "  --vehicle-folder <path>     Path to vehicle folder\n"
              << "  --inst-folder <path>        Path to instance folder\n"
              << "  --num-vehicles <int>        Number of vehicles (must be positive)\n"
              << "  --main-algo <int>           Main algorithm (non-negative integer)\n"
              << "  --sol-mode <int>            Solution mode (non-negative integer)\n"
              << "  --paramfile <string>        Parameter file name\n"
              << "  --scenario <string>         Scenario name\n"
              << "  --save-scratch <int>        Save scratch flag (0 or 1)\n"
              << "  --initial-state <int>       Initial state (non-negative integer)\n\n";

    std::cout << "Optional arguments:\n"
              << "  --instance-name <string>    Specific instance name (if not provided, reads from file)\n"
              << "  --help, -h                  Show this help message\n\n";

    std::cout << "Examples:\n"
              << "  # Use all instances from file:\n"
              << "  " << programName << " --vehicle-folder ./vehicles --inst-folder ./instances \\\n"
              << "                    --num-vehicles 4 --main-algo 1 --sol-mode 0 \\\n"
              << "                    --paramfile AnyParameters --scenario test --save-scratch 1 --initial-state 0\n\n"
              << "  # Use specific instance:\n"
              << "  " << programName << " --vehicle-folder ./vehicles --inst-folder ./instances \\\n"
              << "                    --instance-name test.txt --num-vehicles 4 --main-algo 1 --sol-mode 0\\\n"
              << "                    --paramfile AnyParameters --scenario test --save-scratch 0 --initial-state 1 \n";
}


bool ConfigParser::validateConfig(const PConfig& config) {
    if (config->numVehicles_ <= 0) {
        std::cerr << "Error: Number of vehicles must be positive (got "
                  << config->numVehicles_ << ").\n";
        return false;
    }

    if (config->mainAlgo_ < 0) {
        std::cerr << "Error: Main algorithm must be non-negative (got "
                  << config->mainAlgo_ << ").\n";
        return false;
    }

    if (config->solMode_ < 0) {
        std::cerr << "Error: Solution mode must be non-negative (got "
                  << config->solMode_ << ").\n";
        return false;
    }

    if (config->saveScratch_ > 2) {
        std::cerr << "Error: Save scratch must be less than (got "
                  << config->saveScratch_ << ").\n";
        return false;
    }

    // Check if folders are not empty
    if (config->vehicleFolder_.empty()) {
        std::cerr << "Error: Vehicle folder cannot be empty.\n";
        return false;
    }

    if (config->instFolder_.empty()) {
        std::cerr << "Error: Instance folder cannot be empty.\n";
        return false;
    }

    if (config->paramFile_.empty()) {
        std::cerr << "Error: Parameter file cannot be empty.\n";
        return false;
    }

    if (config->scenario_.empty()) {
        std::cerr << "Error: Scenario name cannot be empty.\n";
        return false;
    }

    if (config->initialState_ < 0) {
        std::cerr << "Error: Initial state must be non-negative (got "
                  << config->initialState_ << ").\n";
        return false;
    }

    return true;
}

bool ConfigParser::parseArguments(int argc, char** argv, PConfig& config) {
    if (argc < 2) {
        std::cerr << "Error: No arguments provided.\n";
        return false;
    }

    std::map<std::string, std::string> args;

    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];

        if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return false;
        }

        if (arg.substr(0, 2) == "--") {
            if (i + 1 >= argc) {
                std::cerr << "Error: Argument " << arg << " requires a value.\n";
                return false;
            }
            args[arg] = argv[i + 1];
            i++; // Skip the next argument as it's the value
        } else {
            std::cerr << "Error: Unknown argument format: " << arg
                      << ". All arguments must start with '--'.\n";
            return false;
        }
    }

    // Check for required arguments
    std::vector<std::string> required = {
        "--vehicle-folder", "--inst-folder", "--num-vehicles", "--main-algo", "--sol-mode",
        "--paramfile", "--scenario", "--save-scratch", "--initial-state"
    };

    for (const auto& req : required) {
        if (args.find(req) == args.end()) {
            std::cerr << "Error: Missing required argument: " << req << "\n";
            return false;
        }
    }

    // Parse and validate values
    try {
        config->vehicleFolder_ = args["--vehicle-folder"];
        config->instFolder_ = args["--inst-folder"];
        config->numVehicles_ = std::stoi(args["--num-vehicles"]);
        config->mainAlgo_ = std::stoi(args["--main-algo"]);
        config->solMode_ = std::stoi(args["--sol-mode"]);
        config->initialState_ = std::stoi(args["--initial-state"]);
        config->paramFile_ = args["--paramfile"];
        config->scenario_ = args["--scenario"];
        config->saveScratch_ = std::stoi(args["--save-scratch"]);
        config->vehicleFileName_ = "vehicles_" + std::to_string(config->numVehicles_) + "_4";

        // Optional instance name
        if (args.find("--instance-name") != args.end()) {
            config->instanceName_ = args["--instance-name"];
        }

    } catch (const std::invalid_argument& e) {
        std::cerr << "Error: Invalid numeric argument. Please check that all numeric "
                  << "arguments contain valid integers.\n";
        return false;
    } catch (const std::out_of_range& e) {
        std::cerr << "Error: Numeric argument out of range. Please use smaller values.\n";
        return false;
    }

    // Validate the configuration
    return validateConfig(config);
}