//
// Created by Ella on 2021-09-06.
//

#ifndef _INPUTPATHS_H
#define _INPUTPATHS_H

#define _SILENCE_EXPERIMENTAL_FILESYSTEM_DEPRECATION_WARNING 1

#include <sstream>
#include <string>
#include <ctime>
#include <experimental/filesystem>
#include <fstream>


//-----------------------------------------------------------------------------
//  getInputDurationData Path Class
//  Instances of this class contain the paths of the inputs
//-----------------------------------------------------------------------------

class InputPaths {
protected:
    // input data addresses
    std::string instanceName_;
    std::string input_TripData_;
    std::string input_InstanceData_;
    std::string input_durationData_;
    std::string input_MIPStart_;
    std::string input_paramFile_;

    // output data addresses
    std::string output_epochISUD_;
    std::string output_epochFinal_;
    std::string output_finalLog_;
    std::string output_solutionLog_;
    std::string output_finalRoutes_;
    std::string output_finalRequests_;
    std::string output_MIPStart_;
    std::string output_offlineRoutes_;
    std::string output_paramFile_;

    double timeOUt = 3600;

public:

    // Constructors
    InputPaths();
    InputPaths(std::string datadir, std::string instanceName, double timeOUt = 3600.0);

    // getters
    const std::string &getInstanceName() const;
    const std::string &getInputTripData() const;
    const std::string &getInputInstanceData() const;
    const std::string &getInputDurationData() const;
    const std::string &getInputMipStart() const;
    const std::string &getInputParamFile() const;

    const std::string &getOutputEpochIsud() const;
    const std::string &getOutputEpochFinal() const;
    const std::string &getOutputFinalLog() const;
    const std::string &getOutputSolutionLog() const;
    const std::string &getOutputFinalRoutes() const;
    const std::string &getOutputFinalRequests() const;
    const std::string &getOutputMipStart() const;
    const std::string &getOutputOfflineRoutes() const;
    const std::string &getOutputParameters() const;


    double getTimeOUt() const;

    // setters
    void setInstanceName(const std::string &instanceName);
    void setTripData(const std::string &tripData);
    void setInstanceData(const std::string &instanceData);
    void setTimeOUt(double timeOUt);

};


#endif //_INPUTPATHS_H
