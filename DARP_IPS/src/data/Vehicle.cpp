//
// Created by Ella on 2021-09-08.
//

#include "Vehicle.h"

//-----------------------------------------------------------------------------
//  vehicle class
//  contains the vehicle info like working time, capacity and passengers
//-----------------------------------------------------------------------------

// Constructor and Destructor
Vehicle::Vehicle(const int vehicleId, int capacity, int startTime, int endTime) : vehicleID_(
        vehicleId), capacity_(capacity), startTime_(startTime), endTime_(endTime) {
    numPassengers_ = 0;
    departTime_ = startTime;
}

Vehicle::~Vehicle() {}

// Setters
void Vehicle::setNumPassengers(int numPassengers) {
    numPassengers_ = numPassengers;
}

void Vehicle::setDepartTime(int departTime) {
    departTime_ = departTime;
}
