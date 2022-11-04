//
// Created by Ella on 2021-09-06.
//

#ifndef INPUTPATHS_H
#define INPUTPATHS_H

#define _SILENCE_EXPERIMENTAL_FILESYSTEM_DEPRECATION_WARNING 1

#include <sstream>
#include <string>
#include <ctime>
#include <filesystem>
#include <fstream>


//-----------------------------------------------------------------------------
//  getInputDurationData Path Class
//  Instances of this class contain the paths of the inputs
//-----------------------------------------------------------------------------

class InputPaths {
protected:
    // input data addresses
    std::string instanceName_;
    std::string instanceDir_;
    std::string outputDir_;
    std::string instanceNameOut_;

    std::string input_TripData_;
    std::string input_InstanceData_;
    std::string input_durationData_;
    std::string input_MIPStart_;
    std::string input_paramFile_;
    std::string input_vehicleFileGeneral_;
    std::string input_vehicleFile_;
    std::string input_onboardsFile_;
    std::string input_waitRequests_;

    // output data addresses
    std::string output_epochISUD_;
    std::string output_epochFinal_;
    std::string output_epochRunTime_;
    std::string output_epochResults_;

    std::string output_solutionLog_;
    std::string output_finalLog_;
    std::string output_finalRoutes_;
    std::string output_finalRequests_;
    std::string output_MIPStart_;
    std::string output_offlineRoutes_;
    std::string output_paramFile_;
    std::string output_onboards_;
    std::string output_waitRequests_;
    std::string output_vehicles_;
    std::string output_instance_;
    std::string output_incDegree_RDCost_;
    std::string output_trip_;


    double timeOut_ = 3600;

public:

    // Constructors
    InputPaths();
    InputPaths(const std::string& datadir, const std::string& instFolder, const std::string& instanceName);

    // getters
    const std::string &getInstanceName() const;
    const std::string &getInputTripData() const;
    const std::string &getInputInstanceData() const;
    const std::string &getInputDurationData() const;
    const std::string &getInputMipStart() const;
    const std::string &getInputParamFile() const;
    const std::string &getInputVehicleFile() const;
    const std::string &getInputVehicleFileGeneral() const;
    const std::string &getInputOnboardsFile() const;
    const std::string &getInputWaitRequests() const;

    const std::string &getOutputEpochIsud() const;
    const std::string &getOutputEpochFinal() const;
    const std::string &getOutputEpochResults() const;

    const std::string &getOutputFinalLog() const;
    const std::string &getOutputSolutionLog() const;
    const std::string &getOutputFinalRoutes() const;
    const std::string &getOutputFinalRequests() const;
    const std::string &getOutputMipStart() const;
    const std::string &getOutputOfflineRoutes() const;
    const std::string &getOutputParameters() const;
    const std::string &getOutputParamFile() const;
    const std::string &getOutputOnboards() const;
    const std::string &getOutputWaitRequests() const;
    const std::string &getOutputVehicles() const;
    const std::string &getOutputInstance() const;
    const std::string &getOutputIncDegreeRdCost() const;
    const std::string &getOutputEpochRunTime() const;


    const std::string &getInstanceNameOut() const;

    const std::string &getOutputTrip() const;


    double getTimeOut() const;

    // setters
    void setInstanceName(const std::string &instanceName);
    void setTripData(const std::string &tripData);
    void setInstanceData(const std::string &instanceData);
    void setTimeOUt(double timeOUt);

    void initializeOutputs(const std::string &algorithm, const std::string &solutionMode);

    // this function create a directory to build a new instance
    void makeInstanceOutput(std::string instNum);
};


#endif //INPUTPATHS_H
