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
    std::string vehicleFolder_;
    std::string vehicleFileName_;
    std::string instFolder_;
    std::string instanceName_;  // optional - if empty, read from file
    int numVehicles_;
    int mainAlgo_;
    int solMode_;
    std::string paramFile_;
    std::string scenario_;
    int saveScratch_;

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
