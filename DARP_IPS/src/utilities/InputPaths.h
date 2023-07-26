//
// Created by Ella on 2021-09-06.
//

#ifndef INPUTPATHS_H
#define INPUTPATHS_H

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
protected:
    // input data addresses
    std::string dataDir_;
    std::string instanceName_;
    std::string instanceDir_;
    std::string outputDir_;
    std::string instanceNameOut_;
    std::string instanceFolder_;

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
    std::string output_subproSize_;
    std::string output_subproRouteTime_;
    std::string output_cplexLog_;
    std::string output_reqDuals_;
    std::string output_vehDuals_;
    std::string output_solutionChange_;


    double timeOut_ = 3600;

public:

    // Constructors
    explicit InputPaths(std::string  datadir, std::string vehicleFile, std::string vehicleFolder);
    InputPaths(std::string  datadir, const std::string& instFolder, const std::string& instanceName,
               std::string vehicleFile, std::string vehicleFolder);

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
    const std::string &getOutputSubproSize() const;
    const std::string &getOutputSubproRouteTime() const;

    const std::string &getOutputCplexLog() const;
    const std::string &getOutputReqDuals() const;
    const std::string &getOutputVehDuals() const;

    const std::string &getOutputSolutionChange() const;

    const std::string &getOutputDir() const;

    double getTimeOut() const;
    void setTimeOUt(double timeOUt);

    // this function defines the path to input data files
    void initializeInputs(const std::string& instFolder, const std::string& instanceName);

    // this function defines the path to outputs
    void initializeOutputs(const std::string &algorithm, const std::string &solutionMode, bool saveScratch);

    // this function create a directory to build a new instance
    void makeInstanceOutput(const std::string& instNum);
};


#endif //INPUTPATHS_H
