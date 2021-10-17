//
// Created by Ella on 2021-09-13.
//

#include "Request.h"


//---------------------------------------------------------------------------------------------
//  request class
//  contains the request info request time, number of passengers, pickup and drop off location
//---------------------------------------------------------------------------------------------

// Constructor and Destructor
Request::Request(const int requestId, float pickUpLatitude, float pickUpLongitude, float dropOffLatitude,
                 float dropOffLongitude, float earlyPick, int nbPassengers, float deltaTime, float minDist,
                 float minTravelTime) : requestID_(requestId), PickUpLatitude_(pickUpLatitude),
                 PickUpLongitude_(pickUpLongitude), DropOffLatitude_(dropOffLatitude),
                 DropOffLongitude_(dropOffLongitude), earlyPick_(earlyPick), nbPassengers_(nbPassengers),
                 deltaTime_(deltaTime), minDist_(minDist), minTravelTime_(minTravelTime) {

    requestStatus_ = NO_ACTION;
    penalty_ = 0;
}

Request::~Request() {}

// Display function
std::string Request::toString() const {
    std::stringstream repStr;
    repStr << "\t" << "# REQUEST ( " << requestID_ << " ) :" << std::endl;
    repStr << "\t" << "# PICKUP_COORDINATE    :" << "(" << PickUpLatitude_ << " , " << PickUpLongitude_ << ")" << std::endl;
    repStr << "\t" << "# DROPOFF_COORDINATE   :" << "(" << DropOffLatitude_ << " , " << DropOffLongitude_ << ")" << std::endl;
    repStr << "\t" << "# NUMBER_OF_PASSENGERS :" << nbPassengers_ << std::endl;
    return repStr.str();
}
