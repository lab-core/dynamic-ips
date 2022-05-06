//
// Created by Ella on 2021-09-06.
//

#include "InputPaths.h"

//-----------------------------------------------------------------------------
//  getInputDurationData Path Class
//  Instances of this class contain the paths of the inputs
//-----------------------------------------------------------------------------

InputPaths::InputPaths() {
    input_TripData_ = "";
    input_InstanceData_ = "";
    instanceName_ = "";
    input_durationData_ = "";
}

InputPaths::InputPaths(std::string datadir, std::string instanceName, double timeOUt)
        : instanceName_(instanceName), timeOUt(timeOUt) {

    std::string instanceDir = datadir + instanceName + "/";

    // create directory for results
    time_t now = time(0);
    tm * curr_tm = localtime(&now);
    char resultFolder[100];
    strftime(resultFolder, 50, "%Y%m%d-%I%M" , curr_tm);
    std::experimental::filesystem::create_directory(instanceDir + "Results_" + resultFolder);

    std::string outputDir = instanceDir + "Results_" + resultFolder + "/";

    //initialize the file names for trip records and instance data
    input_TripData_ = instanceDir + "TRIP_" + instanceName + ".txt";
    input_InstanceData_ = instanceDir + "INSTANCE_" + instanceName + ".txt";
    input_durationData_ = instanceDir + "DURATION_" + instanceName + ".txt";
    input_MIPStart_ = instanceDir + "MIPStart_" + instanceName;
    input_paramFile_ = datadir + "Parameters.txt";
    input_vehicleFile_ = instanceDir + "VEHICLES_" + instanceName + ".txt";
    input_onboardsFile_ = instanceDir + "ONBOARDS_" + instanceName + ".txt";
    input_waitRequests_ = instanceDir + "WaitRequests_" + instanceName + ".txt";

    //initialize the file names for saving outputs
    output_epochISUD_ = outputDir + "epochSolution_" + instanceName + ".csv";
    output_epochFinal_ = outputDir + "finalSolution_" + instanceName + ".csv";
    output_finalLog_ = outputDir + "LogFinalResults_" + instanceName + ".txt";
    output_solutionLog_ = outputDir + "LogSolution" + ".txt";
    output_finalRoutes_ = outputDir + "Routes_" + instanceName + ".csv";
    output_offlineRoutes_ = outputDir + "OfflineRoutes_" + instanceName + ".csv";
    output_finalRequests_ = outputDir + "Requests_" + instanceName + ".csv";
    output_MIPStart_ = outputDir + "MIPStart_" + instanceName;
    output_paramFile_ = outputDir + "Parameters.txt";
    output_onboards_ = outputDir + "ONBOARDS_" + instanceName + ".txt";
    output_waitRequests_ = outputDir + "WaitRequests_" + instanceName + ".txt";
    output_vehicles_ = outputDir + "VEHICLES_" + instanceName + ".txt";
    output_instance_ = outputDir + "INSTANCE_" + instanceName + ".txt";

    // create output files for epoch results
    std::ofstream myFile;
    myFile.open(output_epochISUD_);
//    myFile << "Epoch, ISUDIter,VehicleID,NodeID,RequestTime,ReachTime,NodeType,Latitude,Longitude, LocationID" << std::endl;
    myFile << "Epoch, ISUDIter,VehicleID,NodeID,RequestTime,ReachTime,NodeType,LocationID" << std::endl;
    myFile.close();
    myFile.open(output_epochFinal_);
//    myFile << "Epoch,VehicleID,NodeID,RequestTime,ReachTime,NodeType,Latitude,Longitude, LocationID" << std::endl;
    myFile << "Epoch,VehicleID,NodeID,RequestTime,ReachTime,NodeType, LocationID" << std::endl;
    myFile.close();
}

// getters
const std::string &InputPaths::getInstanceName() const {return instanceName_; }
const std::string &InputPaths::getInputTripData() const {return input_TripData_;}
const std::string &InputPaths::getInputInstanceData() const {return input_InstanceData_; }
const std::string &InputPaths::getInputDurationData() const {return input_durationData_; }
const std::string &InputPaths::getInputMipStart() const { return input_MIPStart_; }
const std::string &InputPaths::getInputParamFile() const { return input_paramFile_; }
const std::string &InputPaths::getInputVehicleFile() const {return input_vehicleFile_;}
const std::string &InputPaths::getInputOnboardsFile() const {return input_onboardsFile_;}
const std::string &InputPaths::getInputWaitRequests() const { return input_waitRequests_;}


const std::string &InputPaths::getOutputEpochIsud() const { return output_epochISUD_;}
const std::string &InputPaths::getOutputEpochFinal() const { return output_epochFinal_; }
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


double InputPaths::getTimeOUt() const {return timeOUt; }

// setters
void InputPaths::setInstanceName(const std::string &instanceName) {
    instanceName_ = instanceName;
}

void InputPaths::setTripData(const std::string &tripData) {
    input_TripData_ = tripData;
}

void InputPaths::setInstanceData(const std::string &instanceData) {
    input_InstanceData_ = instanceData;
}

void InputPaths::setTimeOUt(double timeOUt) {
    InputPaths::timeOUt = timeOUt;
}




















