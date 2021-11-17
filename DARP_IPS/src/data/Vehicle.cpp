//
// Created by Ella on 2021-09-08.
//

#include "Vehicle.h"

//-----------------------------------------------------------------------------
//  vehicle class
//  contains the vehicle info like working time, capacity and passengers
//-----------------------------------------------------------------------------

// Constructor and Destructor
Vehicle::Vehicle(int vehicleId, int capacity, float startTime, float endTime) : vehicleID_(
        vehicleId), capacity_(capacity), startTime_(startTime), endTime_(endTime) {
    numPassengers_ = 0;
    departTime_ = startTime;
    departID_ = Tools::createNodeID(0, SOURCE);
    sinkID_ = Tools::createNodeID(0, SINK);
}

Vehicle::~Vehicle() {}

// Setters
void Vehicle::setDepartTime(float departTime) {
    departTime_ = departTime;
}

// Display function
std::string Vehicle::toString() const {
    std::stringstream repStr;
    repStr << std::left;
    repStr << "# VEHICLE ( " << vehicleID_ << " )" << std::endl;;
    repStr << "#\t" << std::setw(24) << "- VEHICLE_CAPACITY" << " : " << capacity_ << std::endl;
    repStr << "#\t" << std::setw(24) << "- START_TIME (seconds)" << " : " << startTime_ << std::endl;
    repStr << "#\t" << std::setw(24) << "- DEPART_TIME (seconds)" << " : " << departTime_ << std::endl;
    repStr << "#\t" << std::setw(24) << "- END_TIME (seconds)" << " : " << endTime_ << std::endl;
    repStr << "#\t" << std::setw(24) << "- NUMBER_OF_ONBOARDS" << " : " << numPassengers_ << std::endl;
    repStr << "#\t" << std::setw(24) << "- DEPART_NODE_ID" << " : " << departID_ << std::endl;
    repStr << "#" << std::endl;
    return repStr.str();
}

// function to update vehicle depart time at each time and
// update the situation of nodes and ride requests

void Vehicle::updateStatus(int epoch) {
    if ((*currentRoute_)->routeSize_ > 2) {
        int breakIndex = 0;
        for (int i = 1; i < (*currentRoute_)->routeSize_-1; ++i) {
            (*currentRoute_)->routeNodes_[i]->nodeStatus_ = COMMITTED;
            (*currentRoute_)->routeNodes_[i]->reachTime_ = (*currentRoute_)->plannedReachTime_[i];

            // set request status
            if ((*currentRoute_)->routeNodes_[i]->type_ == PICKUP)
                (*(*currentRoute_)->routeNodes_[i]->related_Request_)->requestStatus_ = ON_BOARD;
            else if ((*currentRoute_)->routeNodes_[i]->type_ == DROPOFF)
                (*(*currentRoute_)->routeNodes_[i]->related_Request_)->requestStatus_ = COMPLETED;

            if ((*currentRoute_)->plannedReachTime_[i] >= epoch * epochLength ){

                // at depart point the vehicle is ready to leave the stop location and delta time has passed
                departTime_ = (*currentRoute_)->plannedReachTime_[i] + (*currentRoute_)->routeNodes_[i]->deltaTime_;
                numPassengers_ = (*currentRoute_)->plannedPassengers_[i] + (*currentRoute_)->routeNodes_[i]->nbPassengers_;
                departID_ =  (*currentRoute_)->routeNodes_[i]->nodeID_;
                (*currentRoute_)->routeNodes_[i]->type_ = SOURCE;
                breakIndex = i;
                break;
            }
        }

        for (int i = breakIndex + 1; i < (*currentRoute_)->routeSize_-1; ++i) {
            if ((*(*currentRoute_)->routeNodes_[i]->related_Request_)->requestStatus_ == ON_BOARD) {
                (*currentRoute_)->routeNodes_[i]->nodeStatus_ = PLANNED;
                onboards_.push_back((*currentRoute_)->routeNodes_[i]->nodeID_);
            }
        }
    }

}
