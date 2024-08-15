//
// Created by Ella on 2021-09-13.
//

#include "Request.h"


//---------------------------------------------------------------------------------------------
//  request class
//  contains the request info request time, number of passengers, pickup and drop off location
//---------------------------------------------------------------------------------------------

unsigned int Request::requestCount_ = 0;

// Constructor and Destructor
Request::Request(int pickUpID, int dropOffID, float requestTime, float earlyPick, int nbPassengers, float deltaTime) :
        requestID_(requestCount_++), PickUpID_(pickUpID), requestTime_(requestTime),
        DropOffID_(dropOffID), earlyPick_(earlyPick), nbPassengers_(nbPassengers), serviceTime_(deltaTime){
    allocVehicleID_ = LARGE_CONSTANT;
    initialVehicleID_ = LARGE_CONSTANT;
    solVehicleID_ = LARGE_CONSTANT;
    maxTravelTime_ = 0;
    requestStatus_ = NO_ACTION;
    penalty_ = 0;
    char *name2 = new char[255];
    strncpy(name2, std::to_string(requestID_).c_str(), 255);
    name_ = name2;
    pickTime_ = LARGE_CONSTANT;
    dropTime_ = LARGE_CONSTANT;
    dual_ = 0;
    InitialDual_ = 0;
    minTravelTime_ = 0;
    taskIndex_ = -1;
    taskIncIndex_ = -1;
    taskIndexLabel_ = -1;
}
Request::Request(int pickUpID, int dropOffID, float requestTime, float earlyPick, int nbPassengers, float deltaTime, int pickZoneID, int dropZoneID) :
        requestID_(requestCount_++), PickUpID_(pickUpID), requestTime_(requestTime),
        DropOffID_(dropOffID), earlyPick_(earlyPick), nbPassengers_(nbPassengers), serviceTime_(deltaTime),
        pickZoneID_(pickZoneID), dropZoneID_(dropZoneID){
    allocVehicleID_ = LARGE_CONSTANT;
    initialVehicleID_ = LARGE_CONSTANT;
    solVehicleID_ = LARGE_CONSTANT;
    maxTravelTime_ = 0;
    requestStatus_ = NO_ACTION;
    penalty_ = 0;
    char *name2 = new char[255];
    strncpy(name2, std::to_string(requestID_).c_str(), 255);
    name_ = name2;
    pickTime_ = LARGE_CONSTANT;
    dropTime_ = LARGE_CONSTANT;
    dual_ = 0;
    InitialDual_ = 0;
    minTravelTime_ = 0;
    taskIndex_ = -1;
    taskIncIndex_ = -1;
    taskIndexLabel_ = -1;
}

Request::~Request() {
    delete[] name_;
}

// Getters and Setters
unsigned int Request::getRequestId() const {return requestID_;}
void Request::setMinTravelTime(float minTravelTime) {
    minTravelTime_ = minTravelTime;
}

void Request::setMaxTravelTime(float &alphaParam, float &betaParam) {
    maxTravelTime_ = std::max(alphaParam * minTravelTime_, betaParam + minTravelTime_);
}

// This function update penalties based on elapsed time for any time framework
void Request::setPenalty(float elapsedTime, PParameters &parameters, float simulationStart) {

    penalty_ = static_cast<float>(parameters->deltaPram_
                                  * pow(2, (elapsedTime - (earlyPick_-simulationStart)) / static_cast<float>(10 * parameters->penaltyL_)));
}
// Display function
std::string Request::toString() const {
    std::stringstream repStr;
    repStr << std::left;
    repStr << "# REQUEST ( " << requestID_ << " )" << std::endl;
    repStr << "#\t" << std::setw(24) << "- NUMBER_OF_PASSENGERS" << " : " << nbPassengers_ << std::endl;
    repStr << "#" << std::endl;
    return repStr.str();
}
