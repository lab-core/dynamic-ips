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

    requestStatus_ = NO_ACTION;
    penalty_ = 0;
    readEpoch_ = 0;
    char *name2 = new char[255];
    strncpy(name2, std::to_string(requestID_).c_str(), 255);
    name_ = name2;
    selectStatus_ = NOTSELECTED;
    pickTime_ = MAXReachTime;
    dropTime_ = MAXReachTime;
    dual_ = 0;
    minReachTime_ = 0;
    minTravelTime_ = 0;
}

Request::~Request() {}

// Getters and Setters
void Request::setPenalty(int epoch, PParameters &parameters, float simulationStart) {
    penalty_ = parameters->deltaPram_ * pow(2, ((parameters->epochLength_ * epoch) - (earlyPick_-simulationStart)) / (10 * parameters->epochLength_));
}
const unsigned int Request::getRequestId() const {return requestID_;}
void Request::setMinTravelTime(float minTravelTime) {
    minTravelTime_ = minTravelTime;
}
void Request::setMinReachTime(float minReachTime) {
    minReachTime_ = minReachTime;
}

void Request::setMaxTravelTime(float &alphaParam, float &betaParam) {
    maxTravelTime_ = std::max(alphaParam * minTravelTime_, betaParam + minTravelTime_);
}

// Display function
std::string Request::toString() const {
    std::stringstream repStr;
    repStr << std::left;
    repStr << "# REQUEST ( " << requestID_ << " )" << std::endl;
//    repStr << "#\t" << std::setw(24) << "- PICKUP_COORDINATE" << " : " << "(" << PickUpLatitude_ << " , " << PickUpLongitude_ << ")" << std::endl;
//    repStr << "#\t" << std::setw(24) << "- DROPOFF_COORDINATE" << " : " << "(" << DropOffLatitude_ << " , " << DropOffLongitude_ << ")" << std::endl;
    repStr << "#\t" << std::setw(24) << "- NUMBER_OF_PASSENGERS" << " : " << nbPassengers_ << std::endl;
    repStr << "#" << std::endl;
    return repStr.str();
}










