//
// Created by Ella on 2021-09-06.
//

#include "InputPaths.h"

#include <utility>

//-----------------------------------------------------------------------------
//  getInputDurationData Path Class
//  Instances of this class contain the paths of the inputs
//-----------------------------------------------------------------------------

InputPaths::InputPaths(std::string  datadir) : dataDir_(std::move(datadir)){
    input_TaskData_ = "";
    instanceName_ = "";
    instanceDir_ = "";
    input_durationMatrix_ = "";
    input_durationMatrix_ = dataDir_ + "edge_time_matrix.json";
    input_paramFile_ = dataDir_ + "solver_parameters.json";
}

void InputPaths::initializeInputs() {
    instanceName_ = "Test1";
    instanceDir_ = dataDir_ + instanceName_ + "/";

    //initialize the file names for trip records and instance data
    input_TaskData_ = instanceDir_ + "tasks_by_location.json";
    input_vehicleFile_ = instanceDir_ + "vehicles.json";
}

void InputPaths::initializeOutputs() {
    // create directory for results
    time_t now = time(nullptr);
    tm *curr_tm = localtime(&now);
    char resultFolder[100];
    strftime(resultFolder, 50, "%Y%m%d-%I%M", curr_tm);
    std::string folder_name = instanceDir_ + "_" + resultFolder ;
    char *path = const_cast<char *>(folder_name.c_str());
    if (mkdir(path, 0777) != 0) {
        std::cout << "Output directory can not be created!!!" << std::endl;
        throw myTools::myException("Output directory can not be created!!!", __FILE__,__LINE__);
    }
    outputDir_ = instanceDir_ + "_" + resultFolder + "/";

    //initialize the file names for saving outputs
    output_finalLog_ = outputDir_ + "LogFinalResults_"  + ".txt";
    output_finalRoutes_ = outputDir_ + "Routes_"  + ".csv";
    output_finalTasks_ = outputDir_ + "Tasks_"  + ".csv";
    output_paramCSV_ = outputDir_ + "Parameters_"  + ".csv";
    output_epochRunTime_ = outputDir_ + "epochRuntime_"  + ".csv";
    output_epochResults_ = outputDir_ + "epochResults_"  + ".csv";
    output_subproSize_ = outputDir_ + "epochSubRunTimes_"  + ".csv";
    output_summary_ = outputDir_ + "summary_" + ".csv";
    output_cplexLog_ = outputDir_ + "cplex_" + ".txt";
}









