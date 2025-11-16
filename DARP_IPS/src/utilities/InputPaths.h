//
// Created by Ella on 2021-09-06.
//

#ifndef INPUT_PATHS_H
#define INPUT_PATHS_H

#include <string>
#include <fstream>
#include "utilities/ConfigParser.h"



//-----------------------------------------------------------------------------
//  getInputDurationData Path Class
//  Instances of this class contain the paths of the inputs
//-----------------------------------------------------------------------------

class InputPaths {
protected:
    // input data addresses
    std::string dataDir_;                       // main directory where all instance folders are located
    std::string instanceName_;                  // instance name
    std::string instanceFolder_;                // instance folder which contains a set of instances
    std::string instanceDir_;                   // instance directory
    std::string outputDir_;                     // directory that is created for each run to save outputs
    std::string instanceNameOut_;               // instance name when we are saving the state in the middle
    std::string prefix_;                        // indicate the solution mode and algorithm used to solve

    // input data files
    std::string input_TripData_;                // input trip data file
    std::string input_InstanceData_;            // input instance data file
    std::string input_durationData_;            // input duration data file
    std::string input_paramFile_;               // input parameter file
    std::string input_vehicleFileGeneral_;      // input general vehicle data file
    std::string input_vehicleFile_;             // input specific vehicle data file (small size tests)
    std::string input_onboardsFileGeneral_;     // input general onboards data file
    std::string input_onboardsFile_;            // input specific onboards data file (small size tests)
    std::string input_waitRequests_;            // input wait requests data file
    std::string input_zones_;                   // input zones data file


    // out put save state files

    std::string output_onboards_;               // save the list/info of onboards in a csv
    std::string output_waitRequests_;           // save the list/info of wait requests in a csv
    std::string output_trip_;                   // save the list/info of incoming trips in a csv
    std::string output_vehicles_;               // save the list/state of vehicles in a csv
    std::string output_instance_;               // save the instance data in a txt file

    // output data addresses
    std::string output_epochRunTime_;           // save the summary of each epoch runtime (master, sub problems,..)
    std::string output_epochResults_;           // save the results of solving the Master problem at each epoch
    std::string output_finalLog_;               // save the final routes and parameters and solution in a txt file
    std::string output_finalRoutes_;            // save final routes in a csv
    std::string output_finalRequests_;          // save the status of final requests in a csv
    std::string output_finalVehicles_;          // save the status of final requests in a csv
    std::string output_paramFile_;              // save the parameters
    std::string output_paramCSV_;               // save the parameters csv
    std::string output_incDegree_RDCost_;       // save the reduced cost at each epoch
    std::string output_subproSize_;             // save the information of solving subproblems, nb generated, dominated,...
    std::string output_solverLog_;              // save Cplex log file
    std::string output_reqDuals_;               // save requests duals after each iteration of solving MP
    std::string output_vehDuals_;               // save vehicles duals after each iteration of solving MP
    std::string output_solutionChange_;         // save the changes in incompatibility degree at each epoch
    std::string output_summary_;                // save the summary of the final solution
    std::string output_vehicleEpoch_;           // save the specification of selected routes at each epoch

public:

    // Constructors
    explicit InputPaths(std::string  datadir, PConfig& config);

    // getters
    const std::string &getInstanceName() const;
    const std::string &getInputTripData() const;
    const std::string &getInputInstanceData() const;
    const std::string &getInputDurationData() const;
    const std::string &getInputParamFile() const;
    const std::string &getInputVehicleFile() const;
    const std::string &getInputVehicleFileGeneral() const;
    const std::string &getInputOnboardsFileGeneral() const;
    const std::string &getInputOnboardsFile() const;
    const std::string &getInputWaitRequests() const;
    const std::string &getInputZones() const;

    const std::string &getOutputEpochResults() const;
    const std::string &getOutputFinalLog() const;
    const std::string &getOutputFinalRoutes() const;
    const std::string &getOutputFinalRequests() const;
    const std::string &getOutputFinalVehicles() const;
    const std::string &getOutputParamFile() const;
    const std::string &getOutputParamCsv() const;
    const std::string &getOutputOnboards() const;
    const std::string &getOutputWaitRequests() const;
    const std::string &getOutputVehicles() const;
    const std::string &getOutputInstance() const;
    const std::string &getOutputIncDegreeRdCost() const;
    const std::string &getOutputEpochRunTime() const;
    const std::string &getInstanceNameOut() const;
    const std::string &getOutputTrip() const;
    const std::string &getOutputSubproSize() const;
    const std::string &getOutputSolverLog() const;
    const std::string &getOutputReqDuals() const;
    const std::string &getOutputVehDuals() const;
    const std::string &getOutputSummary() const;
    const std::string &getOutputSolutionChange() const;
    const std::string &getOutputVehicleEpoch() const;

    // this function defines the path to input data files
    void initializeInputs(const std::string& instFolder, const std::string& instanceName);

    // this function defines the path to outputs
    void initializeOutputs(const std::string &algorithm, const std::string &solutionMode, int saveScratch,
        int nbVehicles, const std::string& scenario);

    // this function creates a directory to build a new instance
    void makeInstanceOutput(const std::string& instNum);
};


#endif //INPUT_PATHS_H
