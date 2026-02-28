//
// Created by Elahe Amiri on 2025-06-23.
//

#ifndef CONFIGPARSER_H
#define CONFIGPARSER_H

#include "utilities/MyTools.h"

//-----------------------------------------------------------------------------
//  Configuration structure to hold all program parameters
//-----------------------------------------------------------------------------

struct ProgramConfig {
    std::string vehicleFolder_;     // folder containing vehicle files
    std::string vehicleFileName_;   // vehicle file name
    std::string instFolder_;        // folder containing instance files
    std::string instanceName_;      // instance file name (optional - if empty, read from file)
    int numVehicles_;               // number of vehicles
    int vehicleCapacity_;           // vehicle capacity
    int mainAlgo_;                  // main algorithm
    int solMode_;                   // solution mode: STATIC, DYNAMIC, ANYTIME.
    std::string paramFile_;         // parameter file name
    std::string scenario_;          // scenario name
    int saveScratch_;               // flag to save scratch (0 or 1)
    int initialState_;              // initial state (have onboard/fresh start/etc.)


    // Default constructor with default values
    ProgramConfig();

    // Method to print configuration (for debugging)
    void printConfig() const;
};

//-----------------------------------------------------------------------------
//  Configuration parser class
//-----------------------------------------------------------------------------

class ConfigParser {
public:
    // Parse command line arguments and populate configuration
    static bool parseArguments(int argc, char** argv, PConfig& config);

    // Print usage information
    static void printUsage(const char* programName);


private:
    // Validate numeric arguments
    static bool validateConfig(const PConfig& config);
};



#endif //CONFIGPARSER_H
