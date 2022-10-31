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
    instanceDir_ = "";
    input_durationData_ = "";
}

InputPaths::InputPaths(const std::string& datadir, const std::string& instFolder, const std::string& instanceName)
        : instanceName_(instanceName) {

    instanceDir_ = datadir + instFolder + "/" + instanceName + "/";

    //initialize the file names for trip records and instance data
    input_TripData_ = instanceDir_ + "TRIP_" + instanceName + ".txt";
    input_InstanceData_ = instanceDir_ + "INSTANCE_" + instanceName + ".txt";
 //   input_durationData_ = instanceDir_ + "DURATION_" + instanceName + ".txt";
    input_durationData_ = datadir + "edge_time_matrix.txt";
    input_MIPStart_ = instanceDir_ + "MIPStart_" + instanceName;
    input_paramFile_ = datadir + "Parameters.txt";
    input_vehicleFile_ = instanceDir_ + "VEHICLES_" + instanceName + ".txt";
    input_vehicleFileGeneral_ = datadir + "manhattan-vehicles/vehicles_2000_7.txt";
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
double InputPaths::getTimeOut() const {return timeOut_; }

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
    InputPaths::timeOut_ = timeOUt;
}

void InputPaths::initializeOutputs(const std::string &algorithm, const std::string &solutionMode) {
    // create directory for results
    time_t now = time(nullptr);
    tm * curr_tm = localtime(&now);
    char resultFolder[100];
    strftime(resultFolder, 50, "%Y%m%d-%I%M" , curr_tm);
    std::experimental::filesystem::create_directory(instanceDir_ + solutionMode + "_"+ algorithm + "_" + resultFolder);

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

    // create output files for epoch results
    std::ofstream myFile;
    myFile.open(output_epochISUD_);
    myFile << "Epoch, ISUDIter,VehicleID,NodeID,RequestTime,ReachTime,NodeType,LocationID,RouteID" << std::endl;
    myFile.close();
    myFile.open(output_epochFinal_);
    myFile << "Epoch,VehicleID,NodeID,RequestTime,ReachTime,NodeType, LocationID" << std::endl;
    myFile.close();
    myFile.open(output_incDegree_RDCost_);
    myFile << "Epoch, ISUDIter, VehicleID, IncDegree, ReducedCost, RouteID" << std::endl;
    myFile.close();
    myFile.open(output_epochRunTime_);
    myFile << "Epoch, nbRequests, nbNodes, EpochRuntime, AvgEpochRuntime, ElapsedTime, ISUD_Runtime, RP_Runtime, "
              "CP_Runtime, MIPISUD_Runtime, SubProblemRuntime, PreProcessTime" << std::endl;
    myFile.close();
}

void InputPaths::makeInstanceOutput(std::string instNum) {
    instanceNameOut_ = instanceName_ + "_" + instNum;
    std::experimental::filesystem::create_directory(outputDir_ + instanceNameOut_);
    std::string outputDir = outputDir_ + instanceNameOut_ + "/";
    output_onboards_ = outputDir + "ONBOARDS_" + instanceNameOut_ + ".txt";
    output_waitRequests_ = outputDir + "WaitRequests_" + instanceNameOut_ + ".txt";
    output_vehicles_ = outputDir + "VEHICLES_" + instanceNameOut_ + ".txt";
    output_instance_ = outputDir + "INSTANCE_" + instanceNameOut_ + ".txt";
    output_trip_ = outputDir + "TRIP_" + instanceNameOut_ + ".txt";
}




























