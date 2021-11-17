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
Request::Request(float pickUpLatitude, float pickUpLongitude, float dropOffLatitude,
                 float dropOffLongitude, float earlyPick, int nbPassengers, float deltaTime, float minDist,
                 float minTravelTime) : requestID_(requestCount_++), PickUpLatitude_(pickUpLatitude),
                 PickUpLongitude_(pickUpLongitude), DropOffLatitude_(dropOffLatitude),
                 DropOffLongitude_(dropOffLongitude), earlyPick_(earlyPick), nbPassengers_(nbPassengers),
                 deltaTime_(deltaTime), minDist_(minDist), minTravelTime_(minTravelTime) {

    requestStatus_ = NO_ACTION;
    penalty_ = 0;
    char* name2 = new char[255];
    strncpy(name2, std::to_string(requestID_).c_str(), 255);
    name_ = name2;
}

Request::~Request() {}

// Getters and Setters
void Request::setPenalty(int epoch) {
    penalty_ = deltaPram * pow(2, ((epochLength * epoch) - earlyPick_) / (10 * epochLength));

}
const unsigned int Request::getRequestId() const {return requestID_;}

// Display function
std::string Request::toString() const {
    std::stringstream repStr;
    repStr << std::left;
    repStr << "# REQUEST ( " << requestID_ << " )" << std::endl;
    repStr << "#\t" << std::setw(24) << "- PICKUP_COORDINATE" << " : " << "(" << PickUpLatitude_ << " , " << PickUpLongitude_ << ")" << std::endl;
    repStr << "#\t" << std::setw(24) << "- DROPOFF_COORDINATE" << " : " << "(" << DropOffLatitude_ << " , " << DropOffLongitude_ << ")" << std::endl;
    repStr << "#\t" << std::setw(24) << "- NUMBER_OF_PASSENGERS" << " : " << nbPassengers_ << std::endl;
    repStr << "#" << std::endl;
    return repStr.str();
}




