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
Request::Request(int pickUpID, int dropOffID, float earlyPick, int nbPassengers, float deltaTime) :
                 requestID_(requestCount_++), PickUpID_(pickUpID),
                 DropOffID_(dropOffID), earlyPick_(earlyPick), nbPassengers_(nbPassengers), deltaTime_(deltaTime) {
    vehicleID_ = 0;
    maxTravelTime_ = 0;
    requestStatus_ = NO_ACTION;
    penalty_ = 0;
    char *name2 = new char[255];
    strncpy(name2, std::to_string(requestID_).c_str(), 255);
    name_ = name2;
    pickTime_ = MAXReachTime;
    dropTime_ = MAXReachTime;
    dual_ = 0;
    CPDual_ = 0;
    minTravelTime_ = 0;
    taskIndex_ = requestID_;
}

Request::~Request() = default;

// Getters and Setters
// This function update penalties based on fixed time epochs for rolling horizon
void Request::setPenaltyEpoch(int epoch, PParameters &parameters, float simulationStart) {
    /*penalty_ = static_cast<float>(parameters->deltaPram_
            * pow(2, (static_cast<float>(parameters->epochLength_ * epoch)
            - (earlyPick_-simulationStart)) / static_cast<float>(10 * parameters->epochLength_)));*/
    penalty_ = static_cast<float>(parameters->deltaPram_
                                  * pow(2, (static_cast<float>(parameters->epochLength_ * epoch)
                                            - (earlyPick_-simulationStart)) / static_cast<float>(10 * 30)));
}
// This function update penalties based on elapsed time for any time framework
void Request::setPenalty(float elapsedTime, PParameters &parameters, float simulationStart, float length) {
    /*penalty_ = static_cast<float>(parameters->deltaPram_
            * pow(2, (elapsedTime - (earlyPick_-simulationStart)) / static_cast<float>(10 * length)));*/
    penalty_ = static_cast<float>(parameters->deltaPram_
                                  * pow(2, (elapsedTime - (earlyPick_-simulationStart)) / static_cast<float>(10 * 30)));
}
unsigned int Request::getRequestId() const {return requestID_;}
void Request::setMinTravelTime(float minTravelTime) {
    minTravelTime_ = minTravelTime;
}

void Request::setMaxTravelTime(float &alphaParam, float &betaParam) {
    maxTravelTime_ = std::max(alphaParam * minTravelTime_, betaParam + minTravelTime_);
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












