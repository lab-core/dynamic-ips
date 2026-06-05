//
// Created by Elahe Amiri on 2025-06-23.
//

#include "ConfigParser.h"
#include "utilities/Types.h"
#include "utilities/PlatformCompat.h"
#include <array>
#include <cctype>
#include <iostream>
#include <map>
#include <algorithm>

// ProgramConfig implementation
// numVehicles_ / vehicleCapacity_ default to -1, meaning "not provided on the
// command line — fall back to the instance / vehicle-file values".
ProgramConfig::ProgramConfig()
    : numVehicles_(-1), vehicleCapacity_(-1), mainAlgo_(-1), solMode_(-1),
      initialState_(-1) {}

namespace {
/// @brief Uppercase a copy of a string and strip surrounding whitespace.
std::string normalizeToken(const std::string& text) {
    std::string out;
    for (char c : text)
        if (!std::isspace(static_cast<unsigned char>(c)))
            out.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(c))));
    return out;
}

/// @brief Parse a CLI value that may be either an integer index or one of the
/// given enum names (case-insensitive). Numbers keep backward compatibility.
///
/// @param value Raw command-line value (e.g. "2" or "B_CG").
/// @param names Canonical enum names, indexed by enum value.
/// @return The matching enum index, or -1 if the value is unrecognized.
template <std::size_t N>
int parseEnumArg(const std::string& value, const std::array<const char*, N>& names) {
    try {
        std::size_t pos = 0;
        const int num = std::stoi(value, &pos);
        if (pos == value.size())  // the whole token was numeric
            return (num >= 0 && static_cast<std::size_t>(num) < N) ? num : -1;
    } catch (const std::exception&) {
        // Not an integer — fall through to name matching.
    }
    const std::string token = normalizeToken(value);
    for (std::size_t i = 0; i < N; ++i)
        // Guard against a names array declared larger than its initializer list
        // (trailing entries are null), which would otherwise crash normalizeToken.
        if (names[i] != nullptr && normalizeToken(names[i]) == token)
            return static_cast<int>(i);
    return -1;
}
}  // namespace

void ProgramConfig::printConfig() const {
    std::cout << "Configuration:\n";
    const char* algoName = (mainAlgo_ >= 0 &&
        static_cast<std::size_t>(mainAlgo_) < enum_strings::mainAlgorithmNames.size())
        ? enum_strings::mainAlgorithmNames[mainAlgo_] : "UNKNOWN";
    const char* modeName = (solMode_ >= 0 &&
        static_cast<std::size_t>(solMode_) < enum_strings::solutionModeNames.size())
        ? enum_strings::solutionModeNames[solMode_] : "UNKNOWN";

    std::cout << "  Data directory: " << dataDir_ << "\n";
    std::cout << "  Vehicle folder: " << vehicleFolder_ << "\n";
    std::cout << "  Instance folder: " << instFolder_ << "\n";
    std::cout << "  Number of vehicles: "
              << (numVehicles_ < 0 ? "(from vehicle file)" : std::to_string(numVehicles_)) << "\n";
    std::cout << "  Vehicle capacity: "
              << (vehicleCapacity_ < 0 ? "(from vehicle file)" : std::to_string(vehicleCapacity_)) << "\n";
    std::cout << "  Main algorithm: " << mainAlgo_ << " (" << algoName << ")\n";
    std::cout << "  Solution mode: " << solMode_ << " (" << modeName << ")\n";
    std::cout << "  Parameter file: " << paramFile_ << "\n";
    std::cout << "  Scenario name: " << scenario_ << "\n";
    std::cout << "  Output directory: " << (outputDir_.empty() ? "(local — next to instance data)" : outputDir_) << "\n";
    // When --num-vehicles is omitted the size-specific name is not built here;
    // the solver reads <vehicle-folder>/vehicles.txt once the instance is known.
    std::cout << "  Vehicle file: "
              << (vehicleFileName_.empty() ? "vehicles (fleet read from file)" : vehicleFileName_) << "\n";

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
    std::cout << "Usage: " << programName << " <options>\n"
              << "Running with no options solves the bundled toy example "
                 "(data/ToyExample).\n\n"
              << "Required arguments:\n"
              << "  --inst-folder <path>        Path to instance folder\n"
              << "  --main-algo <int|name>      Main algorithm: 0..4 or a name\n"
              << "                              (GREEDY, MIP, B_CG, F_ICG, A_CG)\n"
              << "  --sol-mode <int|name>       Solution mode: 0..2 or a name\n"
              << "                              (STATIC, DYNAMIC, ANYTIME)\n"
              << "  --paramfile <string>        Parameter file name\n"
              << "  --scenario <string>         Scenario name\n\n";

    std::cout << "Optional arguments:\n"
              << "  --data-dir <path>           Root data directory (default: current directory)\n"
              << "  --vehicle-folder <path>     Vehicle folder (default: vehicles)\n"
              << "  --num-vehicles <int>        Fleet size; selects vehicles_<N>_4.txt.\n"
              << "                              Omit to read the fleet from <folder>/vehicles.txt\n"
              << "  --vehicle-capacity <int>    Vehicle capacity (default: the per-vehicle capacity in the file)\n"
              << "  --initial-state <int>       Fleet state at the start of the simulation (default: 0):\n"
              << "                                0 = fresh start: vehicles idle at their depots, no passengers\n"
              << "                                1 = warm start: load the precomputed fleet state and onboard\n"
              << "                                    passengers from the general ONBOARDS_<file> in the folder\n"
              << "                                2 = resume: continue from a saved mid-simulation state\n"
              << "                                    (instance-specific VEHICLES_/ONBOARDS_/WaitRequests_ files)\n"
              << "  --instance-name <string>    Specific instance name (if not provided, reads from file)\n"
              << "  --output-dir <path>         Root directory for output files (default: next to instance data)\n"
              << "                              Can also be set via the DARP_OUTPUT_DIR environment variable\n"
              << "  --help, -h                  Show this help message\n\n";

    std::cout << "Examples:\n"
              << "  # Minimal run (algo/mode by name, fleet size from the instance):\n"
              << "  " << programName << " --inst-folder ./instances --instance-name test \\\n"
              << "                    --main-algo B_CG --sol-mode DYNAMIC \\\n"
              << "                    --paramfile AnyParameters --scenario test\n\n"
              << "  # Explicit fleet, numeric algo/mode (backward compatible), HPC output:\n"
              << "  " << programName << " --vehicle-folder ./vehicles --inst-folder ./instances \\\n"
              << "                    --instance-name test --num-vehicles 4 --vehicle-capacity 4 \\\n"
              << "                    --main-algo 2 --sol-mode 1 \\\n"
              << "                    --paramfile AnyParameters --scenario test --initial-state 1 \\\n"
              << "                    --output-dir /scratch/myuser/dynamic-ips\n";
}


bool ConfigParser::validateConfig(const PConfig& config) {
    // -1 is the "auto" sentinel (derive from the instance / vehicle file); any
    // other non-positive value is an error.
    if (config->numVehicles_ != -1 && config->numVehicles_ <= 0) {
        std::cerr << "Error: Number of vehicles must be positive (got "
                  << config->numVehicles_ << ").\n";
        return false;
    }

    if (config->vehicleCapacity_ != -1 && config->vehicleCapacity_ <= 0) {
        std::cerr << "Error: Vehicle capacity must be positive (got "
                  << config->vehicleCapacity_ << ").\n";
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

namespace {
/// @brief Check whether a path exists and is a directory.
bool isDirectory(const std::string& path) {
    return platform::pathIsDirectory(path);
}
}  // namespace

bool ConfigParser::loadToyDefaults(const PConfig& config) {
    // The binary may be launched from the repository root, from cpp/, or from
    // the build/bin directory, so probe a few relative locations for the toy
    // data folder and use the first that exists.
    static const std::vector<std::string> kCandidates = {
        "data/ToyExample", "../data/ToyExample",
        "../../data/ToyExample", "../../../data/ToyExample"};

    std::string toyDir;
    for (const auto& candidate : kCandidates) {
        if (isDirectory(candidate)) {
            toyDir = candidate;
            break;
        }
    }
    if (toyDir.empty()) {
        std::cerr << "Error: No arguments provided and the default toy example "
                     "(data/ToyExample) could not be located from the current "
                     "directory.\n";
        return false;
    }

    config->dataDir_ = toyDir;
    config->vehicleFolder_ = "vehicles";
    config->instFolder_ = "Instances_toy";
    config->instanceName_ = "toy";
    config->numVehicles_ = -1;     // take the fleet size from the instance
    config->vehicleCapacity_ = -1;  // take the capacity from the vehicle file
    config->mainAlgo_ = 2;   // B_CG
    config->solMode_ = 1;    // DYNAMIC (B-CG / BatchSolver)
    config->initialState_ = 0;
    config->paramFile_ = toyDir + "/ToyParameters";
    config->scenario_ = "toy";
    config->outputDir_ = toyDir + "/runs";

    std::cout << "No arguments provided — running the built-in toy example from "
              << toyDir << ".\n"
              << "Pass --help to see how to run your own instances.\n\n";
    return true;
}

bool ConfigParser::parseArguments(int argc, char** argv, PConfig& config) {
    if (argc < 2) {
        return loadToyDefaults(config);
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
        "--inst-folder", "--main-algo", "--sol-mode", "--paramfile", "--scenario"
    };

    for (const auto& req : required) {
        if (args.find(req) == args.end()) {
            std::cerr << "Error: Missing required argument: " << req << "\n";
            return false;
        }
    }

    // Parse and validate values
    try {
        config->dataDir_ = args["--data-dir"];
        // --vehicle-folder defaults to "vehicles"
        config->vehicleFolder_ = args.count("--vehicle-folder") ? args["--vehicle-folder"] : "vehicles";
        config->instFolder_ = args["--inst-folder"];
        // --num-vehicles / --vehicle-capacity default to -1 ("auto"): the fleet
        // size is taken from the instance and the capacity from the vehicle file.
        config->numVehicles_ = args.count("--num-vehicles") ? std::stoi(args["--num-vehicles"]) : -1;
        config->vehicleCapacity_ = args.count("--vehicle-capacity") ? std::stoi(args["--vehicle-capacity"]) : -1;

        // --main-algo / --sol-mode accept either an integer or an enum name.
        config->mainAlgo_ = parseEnumArg(args["--main-algo"], enum_strings::mainAlgorithmNames);
        if (config->mainAlgo_ < 0) {
            std::cerr << "Error: Unknown --main-algo value '" << args["--main-algo"]
                      << "'. Use 0..4 or a name (GREEDY, MIP, B_CG, F_ICG, A_CG).\n";
            return false;
        }
        config->solMode_ = parseEnumArg(args["--sol-mode"], enum_strings::solutionModeNames);
        if (config->solMode_ < 0) {
            std::cerr << "Error: Unknown --sol-mode value '" << args["--sol-mode"]
                      << "'. Use 0..2 or a name (STATIC, DYNAMIC, ANYTIME).\n";
            return false;
        }

        // --initial-state defaults to 0 (fresh start).
        config->initialState_ = args.count("--initial-state") ? std::stoi(args["--initial-state"]) : 0;
        config->paramFile_ = args["--paramfile"];
        config->scenario_ = args["--scenario"];
        // The size-specific vehicle file name can only be built now if the fleet
        // size was given; otherwise it is resolved once the instance is read.
        if (config->numVehicles_ > 0)
            config->vehicleFileName_ = "vehicles_" + std::to_string(config->numVehicles_) + "_4";

        // Optional instance name
        if (args.find("--instance-name") != args.end()) {
            config->instanceName_ = args["--instance-name"];
        }

        // Optional output directory: CLI flag takes priority, then env var, then empty (local)
        if (args.find("--output-dir") != args.end()) {
            config->outputDir_ = args["--output-dir"];
        } else {
            const char* envDir = std::getenv("DARP_OUTPUT_DIR");
            config->outputDir_ = (envDir != nullptr) ? envDir : "";
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