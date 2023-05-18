//
// Created by Ella on 2021-09-06.
//

#include "InputPaths.h"

#include <utility>

//-----------------------------------------------------------------------------
//  getInputDurationData Path Class
//  Instances of this class contain the paths of the inputs
//-----------------------------------------------------------------------------

InputPaths::InputPaths(std::string  datadir, std::string vehicleFile, std::string vehicleFolder) : dataDir_(std::move(datadir)){
    input_TripData_ = "";
    input_InstanceData_ = "";
    instanceName_ = "";
    instanceDir_ = "";
    input_durationData_ = "";
    input_durationData_ = dataDir_ + "edge_time_matrix.txt";
    input_paramFile_ = dataDir_ + "Parameters.txt";
    input_vehicleFileGeneral_ = dataDir_ + vehicleFolder + "/" + vehicleFile + ".txt";
}

InputPaths::InputPaths(std::string  datadir, const std::string& instFolder, const std::string& instanceName,
                       std::string vehicleFile, std::string vehicleFolder)
        : dataDir_(std::move(datadir)), instanceName_(instanceName) {

    instanceDir_ = dataDir_ + instFolder + "/" + instanceName + "/";

    //initialize the file names for trip records and instance data
    input_TripData_ = instanceDir_ + "TRIP_" + instanceName + ".txt";
    input_InstanceData_ = instanceDir_ + "INSTANCE_" + instanceName + ".txt";
    //   input_durationData_ = instanceDir_ + "DURATION_" + instanceName + ".txt";
    input_durationData_ = dataDir_ + "edge_time_matrix.txt";
    input_MIPStart_ = instanceDir_ + "MIPStart_" + instanceName;
    input_paramFile_ = dataDir_ + "Parameters.txt";
    input_vehicleFile_ = instanceDir_ + "VEHICLES_" + instanceName + ".txt";
    input_vehicleFileGeneral_ = dataDir_ + vehicleFolder + "/" + vehicleFile + ".txt";
    input_onboardsFile_ = instanceDir_ + "ONBOARDS_" + instanceName + ".txt";
    input_waitRequests_ = instanceDir_ + "WaitRequests_" + instanceName + ".txt";
}

// getters
const std::string &InputPaths::getInstanceName() const {return instanceName_; }
const std::string &InputPaths::getInputTripData() const {return input_TripData_;}
const std::string &InputPaths::getInputInstanceData() const {return input_InstanceData_; }
const std::string &InputPaths::getInputDurationData() const {return input_durationData_; }
const std::string &InputPaths::getInputMipStart() const { return input_MIPStart_; }
const std::string &InputPaths::getInputParamFile() const { return input_paramFile_; }
const std::string &InputPaths::getInputVehicleFile() const {return input_vehicleFile_;}
const std::string &InputPaths::getInputVehicleFileGeneral() const {return input_vehicleFileGeneral_;}
const std::string &InputPaths::getInputOnboardsFile() const {return input_onboardsFile_;}
const std::string &InputPaths::getInputWaitRequests() const { return input_waitRequests_;}


const std::string &InputPaths::getOutputEpochIsud() const { return output_epochISUD_;}
const std::string &InputPaths::getOutputEpochFinal() const { return output_epochFinal_; }
const std::string &InputPaths::getOutputEpochResults() const {return output_epochResults_;}
const std::string &InputPaths::getOutputFinalLog() const { return output_finalLog_; }
const std::string &InputPaths::getOutputSolutionLog() const { return output_solutionLog_; }
const std::string &InputPaths::getOutputFinalRoutes() const { return output_finalRoutes_; }
const std::string &InputPaths::getOutputFinalRequests() const { return output_finalRequests_; }
const std::string &InputPaths::getOutputMipStart() const { return output_MIPStart_; }
const std::string &InputPaths::getOutputOfflineRoutes() const { return output_offlineRoutes_; }
const std::string &InputPaths::getOutputParamFile() const { return output_paramFile_;}
const std::string &InputPaths::getOutputOnboards() const { return output_onboards_;}
const std::string &InputPaths::getOutputWaitRequests() const { return output_waitRequests_;}
const std::string &InputPaths::getOutputVehicles() const { return output_vehicles_;}
const std::string &InputPaths::getOutputInstance() const { return output_instance_;}
const std::string &InputPaths::getOutputIncDegreeRdCost() const {return output_incDegree_RDCost_;}
const std::string &InputPaths::getOutputEpochRunTime() const {return output_epochRunTime_;}
const std::string &InputPaths::getOutputTrip() const {return output_trip_;}
const std::string &InputPaths::getInstanceNameOut() const {return instanceNameOut_;}
const std::string &InputPaths::getOutputSubproSize() const {return output_subproSize_;}
const std::string &InputPaths::getOutputSubproRouteTime() const {return output_subproRouteTime_;}
const std::string &InputPaths::getOutputCplexLog() const {return output_cplexLog_;}
const std::string &InputPaths::getOutputDir() const {return outputDir_;}

double InputPaths::getTimeOut() const {return timeOut_; }

void InputPaths::setTimeOUt(double timeOUt) {InputPaths::timeOut_ = timeOUt;}

void InputPaths::initializeInputs(const std::string &instFolder, const std::string &instanceName) {
    instanceName_ = instanceName;
    instanceDir_ = dataDir_ + instFolder + "/" + instanceName + "/";

    //initialize the file names for trip records and instance data
    input_TripData_ = instanceDir_ + "TRIP_" + instanceName + ".txt";
    input_InstanceData_ = instanceDir_ + "INSTANCE_" + instanceName + ".txt";
    input_MIPStart_ = instanceDir_ + "MIPStart_" + instanceName;
    input_vehicleFile_ = instanceDir_ + "VEHICLES_" + instanceName + ".txt";
    input_onboardsFile_ = instanceDir_ + "ONBOARDS_" + instanceName + ".txt";
    input_waitRequests_ = instanceDir_ + "WaitRequests_" + instanceName + ".txt";
}

void InputPaths::initializeOutputs(const std::string &algorithm, const std::string &solutionMode) {
    // create directory for results
    time_t now = time(nullptr);
    tm * curr_tm = localtime(&now);
    char resultFolder[100];
    strftime(resultFolder, 50, "%Y%m%d-%I%M" , curr_tm);
    std::string folder_name = instanceDir_ + solutionMode + "_"+ algorithm + "_" + resultFolder;
    char *path = const_cast<char *>(folder_name.c_str());
    int check = mkdir(path, 0777);
    outputDir_ = instanceDir_ + solutionMode + "_"+ algorithm + "_" + resultFolder + "/";

    //initialize the file names for saving outputs
    output_epochISUD_ = outputDir_ + "epochSolution_" + instanceName_ + ".csv";
    output_epochFinal_ = outputDir_ + "finalSolution_" + instanceName_ + ".csv";
    output_finalLog_ = outputDir_ + "LogFinalResults_" + instanceName_ + ".txt";
    output_solutionLog_ = outputDir_ + "LogSolution" + ".txt";
    output_finalRoutes_ = outputDir_ + "Routes_" + instanceName_ + ".csv";
    output_offlineRoutes_ = outputDir_ + "OfflineRoutes_" + instanceName_ + ".csv";
    output_finalRequests_ = outputDir_ + "Requests_" + instanceName_ + ".csv";
    output_MIPStart_ = outputDir_ + "MIPStart_" + instanceName_;
    output_paramFile_ = outputDir_ + "Parameters.txt";
    output_incDegree_RDCost_ = outputDir_ + "RouteDegreeCost_" + instanceName_ + ".csv";
    output_epochRunTime_ = outputDir_ + "epochRuntime_" + instanceName_ + ".csv";
    output_epochResults_ = outputDir_ + "epochResults_" + instanceName_ + ".csv";
    output_subproSize_ = outputDir_ + "epochSubRunTimes_" + instanceName_ + ".csv";
    output_subproRouteTime_ = outputDir_ + "epochSubRouteTimes_" + instanceName_ + ".csv";
    output_cplexLog_ = outputDir_ + "LogCplex_" + instanceName_ + ".txt";

    // create output files for epoch results
    std::ofstream myFile;
    myFile.open(output_incDegree_RDCost_);
    myFile << "Epoch, ISUDIter, VehicleID, IncDegree, ReducedCost, CreateTime, RouteID" << std::endl;
    myFile.close();

    myFile.open(output_cplexLog_);
    myFile << instanceName_ << " solved in " << solutionMode << " mode by " << algorithm << std::endl;
    myFile.close();
}

void InputPaths::makeInstanceOutput(const std::string& instNum) {
    instanceNameOut_ = instanceName_ + "_" + instNum;
    std::string folder_name = outputDir_ + instanceNameOut_;
    char *path = const_cast<char *>(folder_name.c_str());
    int check = mkdir(path, 0777);
    std::string outputDir = outputDir_ + instanceNameOut_ + "/";
    output_onboards_ = outputDir + "ONBOARDS_" + instanceNameOut_ + ".txt";
    output_waitRequests_ = outputDir + "WaitRequests_" + instanceNameOut_ + ".txt";
    output_vehicles_ = outputDir + "VEHICLES_" + instanceNameOut_ + ".txt";
    output_instance_ = outputDir + "INSTANCE_" + instanceNameOut_ + ".txt";
    output_trip_ = outputDir + "TRIP_" + instanceNameOut_ + ".txt";
}

