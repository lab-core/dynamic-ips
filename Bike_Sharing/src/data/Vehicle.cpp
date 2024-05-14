//
// Created by Ella on 2021-09-08.
//

#include "Vehicle.h"

#include <utility>

//-----------------------------------------------------------------------------
//  vehicle class
//  contains the vehicle info like working time, capacity and passengers
//-----------------------------------------------------------------------------

// Constructor and Destructor

Vehicle::Vehicle(int vehicleId, const string &name, float readyTime, float endTime, int departIndex, int capacity,
                 int bikeLoad, PNode &departNode) : vehicleID_(vehicleId), name_(name), readyTime_(readyTime),
                 endTime_(endTime), departIndex_(departIndex), capacity_(capacity), bikeLoad_(bikeLoad),
                 departNode_(departNode){
    dual_=0;
    bestReducedCost_ = INFINITY;
}

Vehicle::~Vehicle() = default;

void Vehicle::setCurrentRoute(PRoute &currentRoute) {
    currentRoute_ = currentRoute;
    for (auto & taskObj : currentRoute_->assignedTasks_)
        taskObj->assignedVehicleID_  = vehicleID_;
}

void Vehicle::setEmptyRoute() {
    currentRoute_ = std::make_shared<Route>(vehicleID_);
    currentRoute_->addSource(departNode_, readyTime_, bikeLoad_);
}
