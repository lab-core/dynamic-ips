//
// Created by Ella on 2021-09-06.
//

#include "InputPaths.h"

//-----------------------------------------------------------------------------
//  Input Path Class
//  Instances of this class contain the paths of the inputs
//-----------------------------------------------------------------------------

InputPaths::InputPaths() {
    tripData_ = "";
    instanceData_ = "";
    instanceName_ = "";
}

InputPaths::InputPaths(std::string datadir, std::string instanceName, double timeOUt)
        : instanceName_(instanceName), timeOUt(timeOUt) {

    std::string instanceDir = datadir + instanceName + "/";

    //initialize the file names for trip records and instance data
    tripData_ = instanceDir + "TR_" + instanceName + ".txt";
    instanceData_ = instanceDir + "IN_" + instanceName + ".txt";
}

// getters
const std::string &InputPaths::getInstanceName() const {return instanceName_; }
const std::string &InputPaths::getTripData() const {return tripData_;}
const std::string &InputPaths::getInstanceData() const {return instanceData_; }
double InputPaths::getTimeOUt() const {return timeOUt; }

// setters
void InputPaths::setInstanceName(const std::string &instanceName) {
    instanceName_ = instanceName;
}

void InputPaths::setTripData(const std::string &tripData) {
    tripData_ = tripData;
}

void InputPaths::setInstanceData(const std::string &instanceData) {
    instanceData_ = instanceData;
}

void InputPaths::setTimeOUt(double timeOUt) {
    InputPaths::timeOUt = timeOUt;
}


