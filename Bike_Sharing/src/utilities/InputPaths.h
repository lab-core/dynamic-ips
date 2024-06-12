//
// Created by Ella on 2021-09-06.
//

#ifndef INPUT_PATHS_H
#define INPUT_PATHS_H

#include <sstream>
#include <string>
#include <ctime>
#include <fstream>
#include <sys/stat.h>
#include <iostream>
#include "utilities/MyTools.h"


//-----------------------------------------------------------------------------
//  getInputDurationData Path Class
//  Instances of this class contain the paths of the inputs
//-----------------------------------------------------------------------------

class InputPaths {
public:
    // input data addresses
    std::string dataDir_;                       // main directory where all instance folders are located
    std::string instanceName_;                  // instance name
    std::string instanceDir_;                   // instance directory
    std::string outputDir_;                     // directory that is created for each run to save outputs
    std::string prefix_;                        // indicate the solution mode and algorithm used to solve

    // input data files
    std::string input_TaskData_;
    std::string input_durationMatrix_;
    std::string input_paramFile_;
    std::string input_vehicleFile_;

    // output data addresses
    std::string output_epochRunTime_;           // save the summary of each epoch runtimes (master, sub problems,..)
    std::string output_epochResults_;           // save the results of solving master problem at each epoch
    std::string output_finalLog_;               // save the final routes and parameters and solution in a txt file
    std::string output_finalRoutes_;            // save final routes in a csv
    std::string output_finalTasks_;              // save the status of final requests in a csv
    std::string output_paramCSV_;               // save the parameters csv
    std::string output_subproSize_;             // save the information of solving subproblems, nb generated, dominated,...
    std::string output_summary_;                // save the summary of final solution
    std::string output_cplexLog_;               // save cplex log file


    // Constructors
    explicit InputPaths(std::string datadir);

    // this function defines the path to input data files
    void initializeInputs();

    // this function defines the path to outputs
    void initializeOutputs();

};


#endif //INPUT_PATHS_H
