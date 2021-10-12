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
